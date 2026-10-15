class CameraConfigurationError(ValueError):
    """Invalid camera configuration; independent of the legacy manager."""

# Preserve the local call sites while exposing the camera-owned exception.
DeploymentError = CameraConfigurationError
