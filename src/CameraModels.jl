module CameraModels

using DocStringExtensions
using StaticArrays

# Exports
export AbstractCameraModel
export PinholeCamera
export RadialDistortion

# Abstract types
abstract type AbstractCameraModel end


"""
    $TYPEDEF

Pinhole Camera model is the most simplistic.

Notes
- https://en.wikipedia.org/wiki/Pinhole_camera
- Standard Julia *[Images.jl](https://juliaimages.org/latest/)-frame* convention is, `size(img) <==> (i,j) <==> (height,width) <==> (y,x)`,
  - Common *camera-frame* in computer vision and robotics, `(x,y) <==> (width,height) <==> (j,i)`,
  - Using top left corner of image as `(0,0)` in all cases. 
  - Direct comparison with [OpenCV convention](https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html) is:
    - `(x,y) [CamXYZ] <==> (j,i) [Images.jl] <==> (u,v) [OpenCV]` -- look very carfully at `(u,v) <==> (j,i)` in *image-frame*
- Always follow right hand rule for everything.

DevNotes
- https://en.wikipedia.org/wiki/Distortion_(optics)
"""
struct PinholeCamera{R <: Real} <: AbstractCameraModel
  K::SMatrix{3,3,R}
end

f_w(pc::PinholeCamera) = pc.K[1,1]
f_h(pc::PinholeCamera) = pc.K[2,2]
shear(pc::PinholeCamera) = pc.K[1,2]
c_w(pc::PinholeCamera) = pc.K[1,3]
c_h(pc::PinholeCamera) = pc.K[2,3]

set_f_w!(pc::PinholeCamera, val::Real) = (pc.K[1,1] = val)
set_f_h!(pc::PinholeCamera, val::Real) = (pc.K[2,2] = val)
set_shear!(pc::PinholeCamera, val::Real) = (pc.K[1,2] = val)
set_c_w!(pc::PinholeCamera, val::Real) = (pc.K[1,3] = val)
set_c_h!(pc::PinholeCamera, val::Real) = (pc.K[2,3] = val)

PinholeCamera(; f_w::Real=300,
                f_h::Real=f_w,
                c_w::R=320.0,
                c_h::Real=240,
                shear::Real=0) where {R <: Real} = SMatrix{3,3,R}([f_w shear c_w;
                                                                     0  f_h  c_h;
                                                                     0    0    1])
"""
    $TYPEDSIGNATURES

Constructor helper assuming you just have a camera image and need to start somewhere for a basic camera model.

Notes:
- Calibration will incorrect but hopefully in a distant ballpark to get the calibration process started.
- See [AprilTags.jl Calibration section](https://juliarobotics.org/AprilTags.jl/latest/#Camera-Calibration-1) for code and help. 
"""
function PinholeCamera(img::AbstractArray{T,2}) where T
  f_w, c_w, c_h = size(img, 1), size(img, 2)/2, size(img, 1)/2
  f_h = f_w
  @info "Assuming default PinholeCamera from image $(size(img)):" f_w f_h c_w c_h
  PinholeCamera(f_w=f_w, f_h=f_h, c_w=c_w, c_h=c_h)
end

# Radial distortion model

"""
    $TYPEDEF

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
- TODO make sure LoopVectorization.jl tools like `@avx` is working
"""
struct RadialDistortion{N, R <: Real, K <: AbstractMatrix, C <: AbstractVector}
  Ki::K # SVector{N,R}
  center::C # SVector{2,R} # [h,w]
  # _radius2::Matrix{R} # perhaps SizedArray{R,2} or StaticArray{R,2} or GPUArray{R,2} depending on performance
end

# construction helper on CPU
# TODO better dispatch on Ki conversion to SVector
RadialDistortion(;Ki::AbstractVector{R}=SVector(0.0), 
                  center::AbstractVector{R}=SVector{2,R}(0.0,0.0)) where {R <: Real} = RadialDistortion{length(Ki),R,SVector{length(Ki),R},SVector{2,R}}(Ki isa SVector ? Ki : (@SVector Ki), center)
#

function (rd::RadialDistortion{N,R,K,C})(dest::AbstractMatrix, src::AbstractMatrix) where {N,R,THR}
  # loop over entire image
  for h_d in size(src,1), w_d in size(src,2)
    # temporary coordinates
    @inbounds h_ = h_d - rd.center[1]
    @inbounds w_ = w_d - rd.center[2]
    # calculate the radius from distortion center
    _radius2 = h_^2 + w_^2
    # calculate the denominator
    _denomin = 1
    @inbounds @simd @fastmath for k in 1:N
      _denomin += rd.Ki[k]*(_radius2^k)
    end
    # calculate the new 'undistorted' coordinates and set equal to incoming image
    @inbounds @fastmath h_u = rd.center[1] + h_/_denomin
    @inbounds @fastmath w_u = rd.center[2] + h_/_denomin
    dest[h_u,w_u] = src[h_d,w_d]
  end
  nothing
end


end # module
