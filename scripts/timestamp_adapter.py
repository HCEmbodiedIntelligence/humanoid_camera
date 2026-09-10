#!/usr/bin/env python3
"""ROS message timestamp conversion only. No SDK imports or parameter services."""
from collections import OrderedDict
from copy import deepcopy
from decimal import Decimal
import json
import hashlib
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from realsense2_camera_msgs.msg import Metadata, RGBD
from std_msgs.msg import String
from humanoid_camera.pipeline_diagnostics import PipelineDiagnostics
from humanoid_camera.transport import CaptureSubscription, capture_qos


def ns(header):
    return header.stamp.sec*1_000_000_000+header.stamp.nanosec


def midpoint(metadata):
    """Local offset model anchored by the SDK's global-time mapping per frame."""
    if metadata.get('clock_domain')!='global_time':
        raise ValueError('midpoint mapping needs SDK GLOBAL_TIME metadata')
    sensor=int(metadata['sensor_timestamp'])
    hardware=int(metadata['hw_timestamp'])
    # The raw UVC timestamp is uint32 microseconds; retain the local signed delta.
    delta=(sensor-hardware+2**31)%2**32-2**31
    frame_ns=round(Decimal(str(metadata['frame_timestamp']))*1_000_000)
    return frame_ns+delta*1000,{'id':'sdk_global_local_offset','scale':1.,
        'device_anchor_ns':hardware*1000,'ros_anchor_ns':frame_ns,
        'method':'sdk_global_frame_timestamp_plus_sensor_minus_hw_timestamp',
        'uncertainty_ns':None,'uncertainty_status':'not_measured'}


def set_stamp(header,value):
    header.stamp.sec,header.stamp.nanosec=divmod(value,1_000_000_000)


class TimestampAdapter(Node):
    def __init__(self):
        super().__init__('timestamp_adapter')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('RealSense GLOBAL_TIME normalization requires the ROS system time domain')
        self.source=self.declare_parameter('source_id','front').value
        self.base=self.declare_parameter('driver_prefix','/front/camera').value.rstrip('/')
        config_file=self.declare_parameter('config_file','').value
        self.config_id=hashlib.sha256(Path(config_file).read_bytes()).hexdigest() if config_file else 'configuration_file_unavailable'
        self.limit=self.declare_parameter('buffer_bytes',64*1024*1024).value
        self.pending=OrderedDict();self.lineage=OrderedDict();self.infos={}
        self.bytes=0;self.seq=0;self.epoch=0;self.last_clock=None;self.last_sensor=None
        self.clock_id='realsense:'+uuid.uuid4().hex
        self.cloud_pending=OrderedDict();self.cloud_bytes=0
        self.cloud_limit=min(self.limit,16*1024*1024)
        self.cloud_subscription=None
        self.capture_subscriptions={}
        self.diagnostics=PipelineDiagnostics(self.source,self.clock_id)
        self.diagnostics_pub=self.create_publisher(String,'normalized/diagnostics',qos_profile_sensor_data)
        self.pair_pub=self.create_publisher(RGBD,'normalized/rgbd',capture_qos())
        self.meta_pub=self.create_publisher(Metadata,'normalized/metadata',capture_qos(30))
        self.cloud_pub=self.create_publisher(PointCloud2,'normalized/points',capture_qos(2))
        self.cloud_meta=self.create_publisher(Metadata,'normalized/points_metadata',capture_qos(30))
        for name,topic in [('rgb','color/image_raw'),('depth','depth/image_rect_raw'),
                           ('rgb_meta','color/metadata'),('depth_meta','depth/metadata')]:
            typ=Metadata if name.endswith('_meta') else Image
            self.capture_subscriptions[name]=CaptureSubscription(self,typ,self.base+'/'+topic,
                lambda msg,name=name:self.receive(name,msg),depth=30 if name.endswith('_meta') else 10)
        for name,topic in [('rgb','color/camera_info'),('depth','depth/camera_info')]:
            self.create_subscription(CameraInfo,self.base+'/'+topic,lambda msg,name=name:self.infos.update({name:msg}),qos_profile_sensor_data)
        self.create_timer(.1,self.observe_clock)
        self.create_timer(1.,self.publish_diagnostics)

    def publish_diagnostics(self):
        for subscription in self.capture_subscriptions.values():subscription.refresh()
        self.update_cloud_subscription()
        data=self.diagnostics.snapshot(pending_groups=len(self.pending),pending_bytes=self.bytes,clock_epoch=self.epoch)
        data.update(input_qos={name:sub.state() for name,sub in self.capture_subscriptions.items()},
            output_qos='RELIABLE',cloud_forwarding=self.cloud_subscription is not None,
            cloud_pending_groups=len(self.cloud_pending),cloud_pending_bytes=self.cloud_bytes)
        from rclpy.utilities import get_rmw_implementation_identifier
        data['rmw_implementation']=get_rmw_implementation_identifier()
        self.diagnostics_pub.publish(String(data=json.dumps(data)))

    def update_cloud_subscription(self):
        wanted=self.cloud_pub.get_subscription_count()>0 or self.cloud_meta.get_subscription_count()>0
        if wanted and self.cloud_subscription is None:
            self.cloud_subscription=CaptureSubscription(self,PointCloud2,self.base+'/depth/color/points',
                lambda msg:self.receive('cloud',msg),depth=2)
        elif not wanted and self.cloud_subscription is not None:
            self.cloud_subscription.close();self.cloud_subscription=None
            self.cloud_pending.clear();self.cloud_bytes=0
        if self.cloud_subscription:self.cloud_subscription.refresh()

    def receive_cloud(self,msg):
        key=ns(msg.header)
        if key in self.lineage:
            self.publish_cloud(msg,self.lineage[key]);return
        size=len(msg.data)
        if size>self.cloud_limit:
            self.diagnostics.discard('cloud_oversize',{'cloud':msg});return
        if key in self.cloud_pending:return
        while self.cloud_pending and (self.cloud_bytes+size>self.cloud_limit or len(self.cloud_pending)>=4):
            _,old=self.cloud_pending.popitem(last=False);self.cloud_bytes-=len(old.data)
            self.diagnostics.discard('cloud_buffer_limit',{'cloud':old})
        self.cloud_pending[key]=msg;self.cloud_bytes+=size

    def observe_clock(self):
        now=(self.get_clock().now().nanoseconds,time.monotonic_ns())
        if self.last_clock and abs((now[0]-self.last_clock[0])-(now[1]-self.last_clock[1]))>5_000_000:
            for parts,_ in self.pending.values():self.diagnostics.discard('clock_reset',parts)
            self.epoch+=1;self.pending.clear();self.lineage.clear();self.bytes=0
            self.cloud_pending.clear();self.cloud_bytes=0
        self.last_clock=now

    def receive(self,name,msg):
        self.diagnostics.receive(name)
        if name=='cloud':
            self.receive_cloud(msg);return
        key=ns(msg.header)
        size=len(msg.json_data.encode()) if name.endswith('_meta') else len(msg.data)
        if size>self.limit:
            self.diagnostics.discard('oversize_'+name,{name:msg});return
        while self.pending and (self.bytes+size>self.limit or (key not in self.pending and len(self.pending)>=32)):
            _,(evicted,used)=self.pending.popitem(last=False);self.bytes-=used
            self.diagnostics.discard('buffer_limit',evicted)
        parts,used=self.pending.setdefault(key,({},0))
        if name in parts:return
        parts[name]=msg;self.pending[key]=(parts,used+size);self.bytes+=size
        if not all(k in parts for k in ('rgb','depth','rgb_meta','depth_meta')):return
        del self.pending[key];self.bytes-=used+size
        try:
            raw={k:json.loads(parts[k+'_meta'].json_data) for k in ('rgb','depth')}
            times={k:midpoint(raw[k]) for k in raw}
        except (KeyError,ValueError,TypeError) as error:
            self.diagnostics.discard('invalid_metadata',parts)
            self.get_logger().warning(str(error),throttle_duration_sec=5.)
            return
        sensor=int(raw['rgb']['sensor_timestamp'])
        if self.last_sensor is not None and sensor<self.last_sensor-250_000 and self.last_sensor-sensor<2**31:
            self.epoch+=1;self.lineage.clear()
            self.cloud_pending.clear();self.cloud_bytes=0
        self.last_sensor=sensor
        self.seq+=1
        receive_ros=self.get_clock().now().nanoseconds
        d={'input_contract':'humanoid-ros-capture/1.2','capture_clock_id':'ros','source_id':self.source,
            'source_seq':self.seq,'pair_seq':self.seq,'capture_time_ns':times['rgb'][0],
            'header_timestamp_ns':times['rgb'][0],'source_timestamp_ns':int(raw['rgb']['sensor_timestamp'])*1000,
            'clock_id':self.clock_id,'clock_epoch':self.epoch,'clock_model_id':'sdk_global_local_offset',
            'timestamp_quality':'mapped','clock_uncertainty_ns':None,'clock_evidence':'SDK global-time anchor and raw UVC metadata',
            'receive_time_ns':receive_ros,'receive_clock_id':'ros','receive_steady_ns':time.monotonic_ns(),
            'driver_header_timestamp_ns':key,'pairing_method':'driver_frameset_header',
            'sync_evidence_version':'unverified_driver_association','camera_config_id':self.config_id,
            'calibration_id':'ros_camera_info','exposure_validation':'disabled','valid':True,'invalid_reasons':[]}
        out=RGBD()
        for k in ('rgb','depth'):
            image=parts[k]
            original=image.header
            image.header=deepcopy(original);set_stamp(image.header,times[k][0])
            setattr(out,k,image)
            if k in self.infos:
                info=deepcopy(self.infos[k]);info.header=deepcopy(image.header);setattr(out,k+'_camera_info',info)
            d[k]={'source_seq':int(raw[k]['frame_number']),'source_timestamp_ns':int(raw[k]['sensor_timestamp'])*1000,
                'capture_time_ns':times[k][0],'header_timestamp_ns':times[k][0],'exposure_midpoint_ns':times[k][0],
                'timestamp_semantics':'exposure_midpoint','clock_id':self.clock_id,'clock_epoch':self.epoch,
                'clock_model_id':self.clock_id+'/'+str(self.epoch)+'/'+str(self.seq)+'/'+k,
                'clock_model':{**times[k][1],'id':self.clock_id+'/'+str(self.epoch)+'/'+str(self.seq)+'/'+k},
                'timestamp_quality':'mapped','clock_uncertainty_ns':None,'grid_uncertainty_ns':None,
                'coordinate_frame':original.frame_id,'raw_metadata':raw[k],
                'raw_metadata_json':parts[k+'_meta'].json_data,'original_header_ns':key}
        d['clock_model_id']=d['rgb']['clock_model_id']
        d['calibration']={k:{'k':list(v.k),'d':list(v.d),'r':list(v.r),'p':list(v.p),'width':v.width,'height':v.height,'distortion_model':v.distortion_model,'frame_id':v.header.frame_id} for k,v in self.infos.items()}
        d['calibration_id']=hashlib.sha256(json.dumps(d['calibration'],sort_keys=True).encode()).hexdigest() if d['calibration'] else 'unavailable'
        d['depth'].update(depth_scale=.001,invalid_value=0)
        out.header=deepcopy(out.rgb.header)
        self.meta_pub.publish(Metadata(header=out.header,json_data=json.dumps(d)))
        self.pair_pub.publish(out)
        self.diagnostics.published+=1
        self.lineage[key]=d
        while len(self.lineage)>256:self.lineage.popitem(last=False)
        if key in self.cloud_pending:
            cloud=self.cloud_pending.pop(key);self.cloud_bytes-=len(cloud.data)
            self.publish_cloud(cloud,d)

    def publish_cloud(self,cloud,pair):
        d={k:v for k,v in pair.items() if k not in {'rgb','depth'}}
        d.update(source_id=self.source+'_cloud',camera_source_id=self.source,
            source_depth_seq=pair['depth']['source_seq'],source_rgb_seq=pair['rgb']['source_seq'],
            capture_time_ns=pair['depth']['capture_time_ns'],header_timestamp_ns=pair['depth']['capture_time_ns'],
            source_timestamp_ns=pair['depth']['source_timestamp_ns'],clock_model_id=pair['depth']['clock_model_id'],processing_version='official_ros_pointcloud',
            receive_time_ns=self.get_clock().now().nanoseconds,compute_complete_time_ns=None,
            coordinate_frame=cloud.header.frame_id)
        cloud.header=deepcopy(cloud.header);set_stamp(cloud.header,d['capture_time_ns'])
        self.cloud_meta.publish(Metadata(header=cloud.header,json_data=json.dumps(d)))
        self.cloud_pub.publish(cloud)


def main():
    rclpy.init();node=TimestampAdapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
