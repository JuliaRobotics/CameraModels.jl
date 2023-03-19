
## consolidated functions, first baseline

## =========================================================================================
## Parameter functions

## From yakir12/CameraModels.jl
origin(vector::Union{<:AbstractVector{<:Real},<:Vector3}) = origin3d
origin(ray::Ray) = ray.origin
lookdirection(cameramodel::AbstractCameraModel) = SVector{3}(0,1,0)
updirection(cameramodel::AbstractCameraModel) = SVector{3}(0,0,1)
width(cameramodel::AbstractCameraModel) = cameramodel.width
height(cameramodel::AbstractCameraModel) = cameramodel.height 
direction(vector::Union{<:AbstractVector{<:Real},<:Vector3}) = vector
direction(ray::Ray) = ray.direction
sensorsize(cameramodel::AbstractCameraModel) = SVector{2}(width(cameramodel), height(cameramodel))

"""
    canreproject(camera::CameraModel)

Confirms if point2pixel is implemented for this camera model.
"""
canreproject(camera::AbstractCameraModel) = true


## From JuliaRobotics/Caesar.jl
f_w(pc::AbstractCameraModel) = pc.K[1,1]
f_h(pc::AbstractCameraModel) = pc.K[2,2]
shear(pc::AbstractCameraModel) = pc.K[1,2]
c_w(pc::AbstractCameraModel) = pc.K[1,3]
c_h(pc::AbstractCameraModel) = pc.K[2,3]

set_f_w!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1,1] = val)
set_f_h!(pc::CameraCalibrationMutable, val::Real) = (pc.K[2,2] = val)
set_shear!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1,2] = val)
set_c_w!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1,3] = val)
set_c_h!(pc::CameraCalibrationMutable, val::Real) = (pc.K[2,3] = val)


## computational functions



"""
    $SIGNATURES

Project a world scene onto an image.

Return a transformation that converts real-world coordinates
to camera coordinates. This currently ignores any tangential 
distortion between the lens and the image plane.

Notes
- `pointincamera` is camera's reference frame.

Deprecates:
- yakir12: `point2pixel`

Also see: [`backproject`](@ref)
"""
function project(
    model::AbstractCameraModel,
    pointincamera::Union{<:AbstractVector{<:Real}, <:Point3}
  )
  #
  column = c_w(model) + f_w(model) * pointincamera[1] / pointincamera[3]
  row = c_h(model) - f_h(model) * pointincamera[2] / pointincamera[3]
  return PixelIndex(column, row)
end
## homogeneous point coords xyzw (stereo cameras)
# # xyzw are in the camera frame, i.e. x-columns, y-rows, z-forward
# # left cam
# fz = _f / z
# u = x * fz + center[1] # add center to get PixelCoordinate
# v = y * fz + center[2]
# # right cam
# u2 = (x - w*baseline) * fz + center[1]
# # infront or behind
# valid = (w==0&&0<z) || 0 < (z/w) 



"""
    $SIGNATURES

Backproject from an image into a world scene.

Return a transformation that converts real-world coordinates
to camera coordinates. This currently ignores any tangential 
distortion between the lens and the image plane.

Deprecates:
- yakir12: `pixel2ray`

Also see: [`project`](@ref)
"""
function backproject(
  model::AbstractCameraModel, # AbstractCameraModel, 
  px_coord::Union{<:AbstractVector{<:Real}, <:PixelIndex}
)
  #
  x =  (px_coord[1] - c_w(model)) / f_w(model)
  y = -(px_coord[2] - c_h(model)) / f_h(model)
  return Vector3(x, y, 1)
end
# # camera measurements (u,v), (u2,v)
# lx = (u-center[1])*baseline
# ly = (v-center[2])*baseline
# lz = _f*baseline
# lw = u - u2
# lw<0 ? @warn("backprojecting negative disparity\n") : nothing
# # homogeneous point coords
# return (lz, lx, ly, lw)



## =========================================================================================
## REFACTOR AND CONSOLIDATE BELOW
## =========================================================================================


## From JuliaRobotics/SensorFeatureTracking.jl

function project!(
  ret::AbstractVector{<:Real}, 
  ci::CameraCalibration, #CameraIntrinsic, 
  ce::ArrayPartition, 
  pt::Vector{Float64}
)
  res = ci.K*(ce.R*pt + ce.t)
  ret[1:2] = res[1:2]./res[3]
  nothing
end

project!(
  ret::AbstractVector{<:Real}, 
  cm::CameraModelFull, 
  pt::AbstractVector{<:Real}) = project!(ret, cm.ci, cm.ce, pt)

function project(
  cm::CameraModelFull, 
  pt::AbstractVector{<:Real}
)
  res = Vector{Float64}(2)
  project!(res, cm, pt)
  return res
end

# pinhole camera model
# (x, y)/f = (X, Y)/Z
function cameraResidual!(
    res::AbstractVector{<:Real},
    z::AbstractVector{<:Real},
    ci::CameraCalibration, #CameraIntrinsic,
    ce::ArrayPartition,
    pt::AbstractVector{<:Real}  
  )
  # in place memory operations
  project!(res, ci, ce, pt)
  res[1:2] .*= -1.0
  res[1:2] += z[1:2]
  nothing
end






##