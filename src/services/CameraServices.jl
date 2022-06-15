
## consolidated functions, first baseline

## =========================================================================================
## Parameter functions

## From yakir12/CameraModels.jl
origin(vector::Vector3) = origin3d
origin(ray::Ray) = vector.origin
lookdirection(cameramodel::CameraCalibrationT) = SVector{3}(0,1,0)
updirection(cameramodel::CameraCalibrationT) = SVector{3}(0,0,1)
width(cameramodel::CameraCalibrationT) = cameramodel.width
height(cameramodel::CameraCalibrationT) = cameramodel.height 
direction(vector::Vector3) = vector
direction(ray::Ray) = vector.direction
sensorsize(cameramodel::AbstractCameraModel) = SVector{2}(width(cameramodel), height(cameramodel))

"""
    canreproject(camera::CameraModel)

Confirms if point2pixel is implemented for this camera model.
"""
canreproject(camera::AbstractCameraModel) = true


## From JuliaRobotics/Caesar.jl
f_w(pc::CameraCalibrationT) = pc.K[1,1]
f_h(pc::CameraCalibrationT) = pc.K[2,2]
shear(pc::CameraCalibrationT) = pc.K[1,2]
c_w(pc::CameraCalibrationT) = pc.K[1,3]
c_h(pc::CameraCalibrationT) = pc.K[2,3]

set_f_w!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1,1] = val)
set_f_h!(pc::CameraCalibrationMutable, val::Real) = (pc.K[2,2] = val)
set_shear!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1,2] = val)
set_c_w!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1,3] = val)
set_c_h!(pc::CameraCalibrationMutable, val::Real) = (pc.K[2,3] = val)


## =============================================================================
## computational functions


"""
    $SIGNATURES

Slightly general Radial Distortion type, currently limited to StaticArrays.jl on CPU, but can later be extended to utilize GPUs -- see notes.

Notes
- Make sure `dest` image is large enough to encapsulate the resulting image after un-distortion

Example
```julia
using Images, FileIO, CameraModels

# load the image
img = load("myimg.jpg")

# genereate a radial distortion object
radialdistortion = RadialDistortion()
```

Reference:
From Wikipedia: https://en.wikipedia.org/wiki/Distortion_(optics)
  ( xd ,   yd ) = distorted image point as projected on image plane using specified lens,
  ( xu ,   yu ) = undistorted image point as projected by an ideal pinhole camera,
  ( xc ,   yc ) = distortion center,
              r = sqrt( (xd - xc)^2 + (yd - yc)^2 )

```math
xu = xc + (xd + xc) / (1 + K1*(r^2) + K2*(r^4) + ...)
yu = yc + (yd + yc) / (1 + K1*(r^2) + K2*(r^4) + ...)
```

DevNotes (Contributions welcome):
- TODO manage image clamping if `dest` is too small and data should be cropped out.
- TODO buffer radii matrix for better reuse on repeat image size sequences
- TODO dispatch with either CUDA.jl or AMDGPU.jl <:AbstractArray objects.
- TODO use Tullio.jl with multithreading and GPU
- TODO check if LoopVectorization.jl tools like `@avx` help performance
"""
function radialDistortion!(cc::CameraCalibration{R,N}, dest::AbstractMatrix, src::AbstractMatrix) where {R<:Real,N}
  # loop over entire image
  for h_d in size(src,1), w_d in size(src,2)
    # temporary coordinates
    @inbounds h_ = h_d - cc.center[1]
    @inbounds w_ = w_d - cc.center[2]
    # calculate the radius from distortion center
    _radius2 = h_^2 + w_^2
    # calculate the denominator
    _denomin = 1
    @inbounds @fastmath for k in 1:N
      _denomin += cc.kc[k]*(_radius2^k)
    end
    # calculate the new 'undistorted' coordinates and set equal to incoming image
    @inbounds @fastmath h_u = cc.center[1] + h_/_denomin
    @inbounds @fastmath w_u = cc.center[2] + h_/_denomin
    dest[h_u,w_u] = src[h_d,w_d]
  end
  nothing
end



## 


"""
    $SIGNATURES

Project a world scene onto an image.

Return a transformation that converts real-world coordinates
to camera coordinates. This currently ignores any tangential 
distortion between the lens and the image plane.
"""
function project(
    model::CameraCalibrationT,
    pointincamera::Union{<:AbstractVector{<:Real}, <:Point3}
  )
  #
  column = model.prinicipalpoint[1] + model.focallength[1] * pointincamera[1] / pointincamera[2]
  row = model.prinicipalpoint[2] - model.focallength[2] * pointincamera[3] / pointincamera[2]
  return PixelCoordinate(column, row)
end
## homogeneous point coords xyzw (stereo cameras)
# # left cam
# fz = _f / z
# u = x * fz + center[1]
# v = y * fz + center[2]
# # right cam
# u2 = (x - w*baseline) * fz + center[1]
# # infront or behind
# valid = (w==0&&z>0) || (z/w) > 0 



"""
    $SIGNATURES

Backproject from an image into a world scene.

Return a transformation that converts real-world coordinates
to camera coordinates. This currently ignores any tangential 
distortion between the lens and the image plane.
"""
function backproject(
    model::CameraCalibrationT, 
    pixelcoordinate::Union{<:AbstractVector{<:Real}, <:PixelCoordinate}
  )
  #
  x =  (pixelcoordinate[1] - model.prinicipalpoint[1]) / model.focallength[1]
  z = -(pixelcoordinate[2] - model.prinicipalpoint[2]) / model.focallength[2]
  return Vector3(x, 1, z)
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

function project!(ret::AbstractVector{<:Real}, ci::CameraIntrinsic, ce::CameraExtrinsic, pt::Vector{Float64})
  res = ci.K*(ce.R*pt + ce.t)
  ret[1:2] = res[1:2]./res[3]
  nothing
end

project!(ret::AbstractVector{<:Real}, cm::CameraModelFull, pt::AbstractVector{<:Real}) = project!(ret, cm.ci, cm.ce, pt)

function project(cm::CameraModelFull, pt::AbstractVector{<:Real})
  res = Vector{Float64}(2)
  project!(res, cm, pt)
  return res
end

# pinhole camera model
# (x, y)/f = (X, Y)/Z
function cameraResidual!(
    res::AbstractVector{<:Real},
    z::AbstractVector{<:Real},
    ci::CameraIntrinsic,
    ce::CameraExtrinsic,
    pt::AbstractVector{<:Real}  
  )
  # in place memory operations
  project!(res, ci, ce, pt)
  res[1:2] .*= -1.0
  res[1:2] += z[1:2]
  nothing
end






##