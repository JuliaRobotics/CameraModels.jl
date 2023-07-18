
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

## From JuliaRobotics/Caesar.jl
f_w(pc::AbstractCameraModel) = pc.K[1,1]
f_h(pc::AbstractCameraModel) = pc.K[2,2]
shear(pc::AbstractCameraModel) = pc.K[1,2]
pp_w(pc::AbstractCameraModel) = pc.K[1,3]
pp_h(pc::AbstractCameraModel) = pc.K[2,3]

set_f_w!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1,1] = val)
set_f_h!(pc::CameraCalibrationMutable, val::Real) = (pc.K[2,2] = val)
set_shear!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1,2] = val)
set_pp_w!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1,3] = val)
set_pp_h!(pc::CameraCalibrationMutable, val::Real) = (pc.K[2,3] = val)

"""
    canreproject(camera::CameraModel)

Confirms if point2pixel is implemented for this camera model.
"""
canreproject(camera::AbstractCameraModel) = true


## computational functions


## =========================================================================================
## FROM SCENE IN FRONT OF CAMERA TO IMAGE -- I.E. PROJECT
## =========================================================================================


## From JuliaRobotics/SensorFeatureTracking.jl
function project!(
  ret::AbstractVector{<:Real}, 
  ci::CameraCalibration, #CameraIntrinsic, 
  c_T_r::ArrayPartition,
  r_P::AbstractVector{<:Real}
)
  res = ci.K*(c_T_r.x[2]*r_P + c_T_r.x[1])
  ret[1:2] ./= res[3]
  PixelIndex(ret[1], ret[2])
end

"""
    $SIGNATURES

Project a world scene onto an image.

Return a transformation that converts real-world coordinates
to camera coordinates. This currently ignores any tangential 
distortion between the lens and the image plane.

Notes
- `r_P` is a point in reference frame tranformed the camera's reference frame:
  - `c_P = c_T_r * r_P`

Deprecates:
- yakir12: `point2pixel`: @deprecate point2pixel(model, pt) project(model, pt[[1;3;2]])

Also see: [`backproject`](@ref)
"""
function project(
    model::AbstractCameraModel,
    r_P::Union{<:AbstractVector{<:Real}, <:Point3};
    c_T_r = ArrayPartition( SVector(0.,0.,0.), SMatrix{3,3}(1.,0.,0.,0.,1.,0.,0.,0.,1.) )
  )
  #
  ret = MVector(0.,0.)
  project!(ret, model, c_T_r, r_P)
  # column = pp_w(model) + f_w(model) * c_P[1] / c_P[3]
  # row = pp_h(model) - f_h(model) * c_P[2] / c_P[3]
  # return PixelIndex(column, row)
end

project!(
  ret::AbstractVector{<:Real}, 
  cm::CameraModelFull, 
  pt::AbstractVector{<:Real}) = project!(ret, cm.ci, cm.ce, pt)


## homogeneous point coords xyzw (stereo cameras)
# xyzw are in the camera frame (c_), i.e. x-columns, y-rows, z-forward
function projectHomogeneous(
  cam::AbstractCameraModel,
  c_Ph::AbstractVector,
)
  # left cam
  x,y,z,w = (c_Ph...,)
  fx_z = f_w(cam) / z
  fy_z = f_h(cam) / z
  col = x * fx_z + pp_w(cam) # add center to get PixelIndex
  row = y * fy_z + pp_h(cam)
  # infront or behind
  depth=z/w
  PixelIndex(row, col; depth, valid = (w==0&&0<z) || 0 < depth)
end
# # right cam
# u2 = (x - w*baseline) * fz + center[1]



## =========================================================================================
## FROM IMAGE TO SCENE IN FRONT OF CAMERA -- I.E. BACKPROJECT
## =========================================================================================


"""
    $SIGNATURES

Backproject from an image into a world scene.

Return a transformation that converts real-world coordinates
to camera coordinates. This currently ignores any tangential 
distortion between the lens and the image plane.

Deprecates:
- yakir12: `pixel2ray`: @deprecate pixel2ray(model, px) backproject(model, px)[[1;3;2]]

Also see: [`project`](@ref)
"""
function backproject(
    model::AbstractCameraModel, 
    px_coord::Union{<:AbstractVector{<:Real}, <:PixelIndex}
  )
  #
  col =  (px_coord[1] - pp_w(model)) / f_w(model)
  row = -(px_coord[2] - pp_h(model)) / f_h(model)
  return Vector3(col, row, 1)
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
## RESIDUAL FUNCTION FOR OPTIMIZATION OR LOSS
## =========================================================================================


# pinhole camera model
# (x, y)/f = (X, Y)/Z
function cameraResidual!(
    res::AbstractVector{<:Real},
    z::AbstractVector{<:Real},
    ci::CameraCalibration, #CameraIntrinsic,
    ce::ArrayPartition,
    pt::Union{PixelIndex,<:AbstractVector{<:Real}},  
  )
  # in place memory operations
  project!(res, ci, ce, pt)
  res[1:2] .*= -1.0
  res[1:2] += z[1:2]
  nothing
end






##