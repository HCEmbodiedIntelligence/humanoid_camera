# HC independent hardware path

The startup/controller helpers and camera settings/scalar validation were migrated from
humanoid_manager commit 26cbe8b7b751276cbcbc2e21d1ae2d1e627a7495. Keep the source
repository attribution and its declared package license (Proprietary); this migration
makes no claim to relicense those routines. Camera-owned configuration errors are
CameraConfigurationError. The new hardware foundation imports neither manager nor HC
nor camera. Camera configuration and launch use that foundation.

Legacy registered_robot/start_robot/release packaging remain optional compatibility
entrypoints and require the legacy manager separately. They are not dependencies of
the independent camera/ensure_controllers path. Existing RealSense local patches,
atomic messages, exposure, depth and point-cloud behavior remain in their source tree.
