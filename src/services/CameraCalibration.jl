



"""
    $SIGNATURES

Constructor helper assuming you just have a camera image and need to start somewhere for a basic camera model.

Notes:
- Calibration will incorrect but hopefully in a distant ballpark to get the calibration process started.
- See [AprilTags.jl Calibration section](https://juliarobotics.org/AprilTags.jl/latest/#Camera-Calibration-1) for code and help. 
"""
function CameraCalibrationMutable(img::AbstractMatrix{T}) where T
  height, width = size(img)
  # emperical assumption usually seen for focal length
  f_w = 1.1*height
  f_h = f_w
  c_w, c_h = width/2, height/2
  K = MMatrix{3,3}([[f_w;0.0;c_w]';[0.0;f_h;c_h]';[0.0;0;1.]'] )
  @info "Assuming default CameraCalibrationMutable from image size(img)=(rows,cols)=$(size(img)):" f_w f_h c_w c_h
  CameraCalibrationMutable(;width, height, K)
end



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
function radialDistortion!(
  cc::CameraCalibration{<:Real,N}, 
  dest::AbstractMatrix, 
  src::AbstractMatrix
) where N
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
