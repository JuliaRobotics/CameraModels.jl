
# NOTICE

This package is meant to consolidate the camera models previously found in the Julia package ecosystem.  There are still many gaps and community contributions are encouranged.

# CameraModels.jl
Basic Camera Models including pinhole, radial distortion etc.

> Click on badges to follow links:

| Stable Release | Dev branch | Coverage | Documentation |
|----------------|------------|----------|---------------|
| [![cms-ci-stb][cms-ci-stb-img]][cms-ci-stb-url] <br> [![version][cms-ver-img]][cms-rel-url] | [![cms-ci-dev-img]][cms-ci-dev-url] | [![cms-cov-img]][cms-cov-url] | [![cjl-slack-badge]][cjl-slack] <br> [![caesar-docs]][cjl-docs-url] |

# Roadmap

Project organization is currently done here:
- [https://github.com/orgs/JuliaRobotics/projects/9/views/1](https://github.com/orgs/JuliaRobotics/projects/9/views/1)

## TODO List

- [x] Copy existing camera code from Caesar.jl and RoME.jl here,
- [x] Functional tests for consolidated CameraCalibration type
- [x] Implement radial distortion computations,
- [ ] Numerical tests for pinhole camera model,
- [ ] Integrate downstream with packages like AprilTags.jl and Caesar.jl
- [ ] Tests for radial distortion model,
- [ ] Homogeneous coordinates model,
- [ ] ...


### v0.1.0

- Created consolidated `CameraCalibration` type to replace various previous camera models (all doing about the same thing),
 - Also `CameraCalibrationMutable` and `CameraCalibratonT` as the Union dispatch type.
- Backward compatibility and deprecations for yakir12's Pinhole camera model.
- Backward compatibility and deprecations for Caesar.jl's PinholeCamera model.
- Backward compatibility and deprecations for SensorFeatureTracking CameraIntrinsics model.
- Backward compatibility and deprecations for CameraModelandParameters.
- Work in progress to consolidate CameraModelFull from JuliaRobotics, but some hangups on where to put CameraExtrinsics.
- Implement but not fully wired up to projections of `radialDistorion` functionality.



[cms-ci-dev-img]: https://github.com/JuliaRobotics/CameraModels.jl/actions/workflows/ci.yml/badge.svg
[cms-ci-dev-url]: https://github.com/JuliaRobotics/CameraModels.jl/actions/workflows/ci.yml
[cms-ci-stb-img]: https://github.com/JuliaRobotics/CameraModels.jl/actions/workflows/ci.yml/badge.svg?branch=release%2Fv0.26
[cms-ci-stb-url]: https://github.com/JuliaRobotics/CameraModels.jl/actions/workflows/ci.yml
[cms-ver-img]: https://juliahub.com/docs/CameraModels/version.svg
[cms-rel-url]: https://github.com/JuliaRobotics/CameraModels.jl/releases
[cms-milestones]: https://github.com/JuliaRobotics/CameraModels.jl/milestones
[cms-cov-img]: https://codecov.io/github/JuliaRobotics/CameraModels.jl/coverage.svg?branch=master
[cms-cov-url]: https://codecov.io/github/JuliaRobotics/CameraModels.jl?branch=master

[caesar-docs]: https://img.shields.io/badge/CaesarDocs-latest-blue.svg
[cjl-docs-url]: http://juliarobotics.github.io/Caesar.jl/latest/

[cjl-slack-badge]: https://img.shields.io/badge/Caesarjl-Slack-green.svg?style=popout
[cjl-slack]: https://join.slack.com/t/caesarjl/shared_invite/zt-ucs06bwg-y2tEbddwX1vR18MASnOLsw
