module CameraModels

using StaticArrays

export AbstractCameraModel
export PinholeCamera

abstract type AbstractCameraModel end


"""
    $TYPEDEF

Pinhole Camera model is the most simplistic camera model assuming an ideal pinhole camera model.

Notes
- https://en.wikipedia.org/wiki/Pinhole_camera
"""
struct PinholeCamera <: AbstractCameraModel
  K::SMatrix{3,3,Float64}
end

centerHeight(pc::PinholeCamera) = pc.K[1,3]
centerWidth(pc::PinholeCamera) = pc.K[2,3]
focalHeight(pc::PinholeCamera) = pc.K[1,1]
focalWidth(pc::PinholeCamera) = pc.K[2,2]


end # module
