

"""
    CameraModel(width,height,fc,cc,skew,kc)

Constructor helper for creating a camera model.
"""
function CameraSkewDistortion(width,height,fc,cc,skew,kc)
    KK = [fc[1]      skew  cc[1];
             0       fc[2] cc[2];
             0		    0     1]
    # KK = [fc[1] skew*fc[1] cc[1];
    #          0       fc[2] cc[2];
    #          0		    0     1]
    Ki = inv(KK)
    CameraModelandParameters(width,height,fc,cc,skew,kc,KK,Ki)
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

# TODO, store radialDistortion in CameraCalibration
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




function intersectLineToPlane3D(
    planenorm::AbstractVector{<:Real}, 
    planepnt::AbstractVector{<:Real}, 
    raydir::AbstractVector{<:Real}, 
    raypnt::AbstractVector{<:Real}
)
    ndotu = dot(planenorm, raydir)
    if ndotu ≈ 0 error("no intersection or line is within plane") end

    w  = raypnt - planepnt
    si = -dot(planenorm, w) / ndotu
    ψ  = w .+ si .* raydir .+ planepnt
    return ψ
end

"""
    $SIGNATURES

Ray trace from pixel coords to a floor in local level reference which is assumed 
aligned with gravity.  Returns intersect in local level frame (coordinates).

Notes
- Implemented against local level to allow easier local or world reference usage,
  - Just assume world is local level, i.e. `l_nFL = w_nFL` and `l_FL = w_FL`.
  - User must provide (assumed dynamic) local level transform via `l_T_ex` -- see example below!
- Coordinate chain used is from ( pixel-array (a) --> camera (c) --> extrinsic (ex) --> level (l) )
  - `c_H_a` from pixel-array to camera (as homography matrix) 
  - `a_F` feature in array pixel frame
  - `l_T_ex` extrinsic in body (or extrinsic to local level), SE3 Manifold element using ArrayPartition
  - `ex_T_c` camera in extrinsic (or camera to extrinsic)
- line-to-plane intersect computation done in the local level frame

Example
```
# Assume body coords x,y,z == fwd,prt,upw

# Camera setup
f = 800.0        # pixels
ci,cj = 360,640  # assuming 720x1280 image
# going from imaging array to camera frame
c_H_a = [0 1 -cj; 1 0 -ci; 0 0 f] # camera matrix

# body to extrinsic of camera -- e.g. camera looking down 0.2 and left 0.2
# local level to body to extrinsic transform 
l_T_b = ArrayPartition([0;0;0.], R0)
b_T_ex = ArrayPartition([0;0;0.], exp_lie(Mr, hat(Mr, R0, [0;0.2;0.2])))
l_T_ex = compose(M, l_T_b, b_T_ex) # this is where body reference is folded in.

# FLOOR in local level coordinates, normal and point
l_nFL = [0; -0.05; 1.]
l_FL = [0; 0; -2.]

# Ray trace to plane
l_Forb = intersectRayToPlane(
  c_H_a,
  a_Forb,
  l_nFL,
  l_FL;
  l_T_ex
)
```

See also: `CameraModels.intersectLineToPlane3D`
"""
function intersectRayToPlane(
    c_H_a::AbstractMatrix{<:Real},
    a_F::AbstractVector{<:Real},
    l_nFL::AbstractVector{<:Real},
    l_FL::AbstractVector{<:Real};
    M = SpecialEuclidean(3),
    R0 = [1 0 0; 0 1 0; 0 0 1.],
    l_T_ex = ArrayPartition([0;0;0.], exp_lie(M.manifold[2], hat(M.manifold[2], R0, [0;0.2;0.]))),
    ex_T_c = ArrayPartition([0;0;0.], [0 0 1; -1 0 0; 0 -1 0.]),
)
    # camera in level (or camera to level) manifold element as ArrayPartition
    l_T_c = compose(M, l_T_ex, ex_T_c)
    
    ## convert pixel location to ray in camera then level frame
    c_F = c_H_a * a_F
    # unit vector ray direction from camera in camera coordinates 
    c_uV = c_F ./ norm(c_F)
    # get ray direction in level coordinates
    l_uV = l_T_c.x[2] * c_uV
    
    # and ray position from camera center in level frame
    l_P = l_T_c.x[1]
    
    ## planar floor surface projection
    l_nFL_ = l_nFL ./ norm(l_nFL)
    
    intersectLineToPlane3D(l_nFL_, l_FL, l_uV, l_P)  # returns intersect in local level.
end

