# History of CameraModels.jl

## v0.1.0

- Created consolidated `CameraCalibration` type to replace various previous camera models (all doing about the same thing),
 - Also `CameraCalibrationMutable` and `CameraCalibratonT` as the Union dispatch type.
- Backward compatibility and deprecations for yakir12's Pinhole camera model.
- Backward compatibility and deprecations for Caesar.jl's PinholeCamera model.
- Backward compatibility and deprecations for SensorFeatureTracking CameraIntrinsics model.
- Backward compatibility and deprecations for CameraModelandParameters.
- Work in progress to consolidate CameraModelFull from JuliaRobotics, but some hangups on where to put CameraExtrinsics.
- Implement but not fully wired up to projections of `radialDistorion` functionality.