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
        self.pair_pub=self.create_publisher(RGBD,'normalized/rgbd',qos_profile_sensor_data)
        self.meta_pub=self.create_publisher(Metadata,'normalized/metadata',qos_profile_sensor_data)
        self.cloud_pub=self.create_publisher(PointCloud2,'normalized/points',qos_profile_sensor_data)
        self.cloud_meta=self.create_publisher(Metadata,'normalized/points_metadata',qos_profile_sensor_data)
        for name,topic in [('rgb','color/image_raw'),('depth','depth/image_rect_raw'),
                           ('rgb_meta','color/metadata'),('depth_meta','depth/metadata'),('cloud','depth/color/points')]:
            typ=Metadata if name.endswith('_meta') else PointCloud2 if name=='cloud' else Image
            self.create_subscription(typ,self.base+'/'+topic,lambda msg,name=name:self.receive(name,msg),qos_profile_sensor_data)
        for name,topic in [('rgb','color/camera_info'),('depth','depth/camera_info')]:
            self.create_subscription(CameraInfo,self.base+'/'+topic,lambda msg,name=name:self.infos.update({name:msg}),qos_profile_sensor_data)
        self.create_timer(.1,self.observe_clock)

    def observe_clock(self):
        now=(self.get_clock().now().nanoseconds,time.monotonic_ns())
        if self.last_clock and abs((now[0]-self.last_clock[0])-(now[1]-self.last_clock[1]))>5_000_000:
            self.epoch+=1;self.pending.clear();self.lineage.clear();self.bytes=0
        self.last_clock=now

    def receive(self,name,msg):
        key=ns(msg.header)
        if name=='cloud' and key in self.lineage:
            self.publish_cloud(msg,self.lineage[key]);return
        size=len(msg.json_data.encode()) if name.endswith('_meta') else len(msg.data)
        if size>self.limit:return
        while self.pending and (self.bytes+size>self.limit or (key not in self.pending and len(self.pending)>=32)):
            _,(_,used)=self.pending.popitem(last=False);self.bytes-=used
        parts,used=self.pending.setdefault(key,({},0))
        if name in parts:return
        parts[name]=msg;self.pending[key]=(parts,used+size);self.bytes+=size
        if not all(k in parts for k in ('rgb','depth','rgb_meta','depth_meta')):return
        del self.pending[key];self.bytes-=used+size
        try:
            raw={k:json.loads(parts[k+'_meta'].json_data) for k in ('rgb','depth')}
            times={k:midpoint(raw[k]) for k in raw}
        except (KeyError,ValueError,TypeError) as error:
            self.get_logger().warning(str(error),throttle_duration_sec=5.)
            return
        sensor=int(raw['rgb']['sensor_timestamp'])
        if self.last_sensor is not None and sensor<self.last_sensor-250_000 and self.last_sensor-sensor<2**31:
            self.epoch+=1;self.lineage.clear()
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
        self.lineage[key]=d
        while len(self.lineage)>256:self.lineage.popitem(last=False)
        if 'cloud' in parts:self.publish_cloud(parts['cloud'],d)

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
