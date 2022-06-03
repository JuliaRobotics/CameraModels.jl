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
Base.@kwdef struct RadialDistortion{N, R <: Real, K <: AbstractVector, C <: AbstractVector}
  Ki::K = SVector(0.0) # SVector{N,R}
  center::C = SVector{2,R}(0.0,0.0) # SVector{2,R} # [h,w]
  # _radius2::Matrix{R} # perhaps SizedArray{R,2} or StaticArray{R,2} or GPUArray{R,2} depending on performance
end


#