

## ================================================================================================
## consolidated types below

export CameraModel # being replaced by AbstractCameraModel
CameraModel = (@warn("CameraModels.CameraModel is deprecated, use CameraModels.AbstractCameraModel instead");AbstractCameraModel)
# abstract type CameraModel end 


@deprecate columns(w...;kw...) width(w...;kw...)
@deprecate rows(w...;kw...) height(w...;kw...)

sensorsize(cameramodel::CameraModel) = SVector{2}(width(cameramodel), height(cameramodel))


export CameraModelandParameters
const CameraModelandParameters = (@warn("CameraModels.CameraModelandParameters is deprecated, use CamereModels.CameraCalibration instead.");CameraCalibration)

function CameraModelandParameters(
    width::Int,
    height::Int,
    fc::AbstractVector{<:Real},
    cc::AbstractVector{<:Real},
    skew::Real,
    kc::AbstractVector{<:Real},
    K::AbstractMatrix{<:Real}  = [[fc[1];skew;cc[1]]';[0.0;fc[2];cc[2]]';[0.0;0;1]'], # legacy constructor
    Ki::AbstractMatrix{<:Real} = inv(K)
  )
  #
  @warn("CameraModels.CameraModelandParameters is deprecated, use CamereModels.CameraCalibration instead.")
  CameraCalibration(;width,height,K)
end


function Base.getproperty(x::CameraModelandParameters, f::Symbol)
  if f == :skew
    getfield(x, :K)[1,2]
  elseif f == :columns
    getfield(x, :width)
  elseif f == :rows
    getfield(x, :height)
  elseif f == :prinicipalpoint || f == :cc
    SA[(getfield(x, :K)[1:2, 3])...]
  elseif f == :focallength || f == :fc
    K = getfield(x, :K)
    SA[K[1,1];K[2,2]] 
  else
    getfield(x, f)
  end
end


# """
# Data structure for a Camera model with parameters.
# Use `CameraModel(width,height,fc,cc,skew,kc)` for easy construction.
# """
# struct CameraModelandParameters <: AbstractCameraModel
#     width::Int		# image width
#     height::Int		# image height
#     fc::Vector{Float64}	# focal length in x and y
#     cc::Vector{Float64}	# camera center
#     skew::Float64	    # skew value
#     kc::Vector{Float64} # distortion coefficients up to fifth order
#     K::Matrix{Float64} # 3x3 camera calibration matrix (Camera intrinsics)
#     Ki::Matrix{Float64} # inverse of a 3x3 camera calibratio matrix
# end


export CameraIntrinsic

# Base.@kwdef struct CameraIntrinsic
#   K::Array{Float64,2}
# end
# CameraIntrinsic(;x0=320.0,y0=240.0,fx=510.0,fy=510.0,s=0.0) = CameraIntrinsic([[fx;s;x0]';[0.0;fy;y0]';[0.0;0;1]'])

## From JuliaRobotics/Caesar.jl
const CameraIntrinsic = (@warn("CameraModels.CameraIntrinsic is deprecated, use CamereModels.CameraCalibration instead.");CameraCalibration)
function CameraIntrinsic(
    K_::AbstractMatrix=[[510;0;320.0]';[0.0;510;240]';[0.0;0;1]']; # legacy constructor
    x0=320.0,y0=240.0,fx=510.0,fy=510.0,s=0.0,                     # legacy function support
    K::AbstractMatrix=[[fx;s;x0]';[0.0;fy;y0]';[0.0;0;1]'],        # consolidated matrix K
    width::Int=round(Int, K[1,3]*2), 
    height::Int=round(Int, K[2,3]*2),
  )
  #
  @warn "CameraModels.CameraIntrinsic is deprecated, use CamereModels.CameraCalibration instead."
  CameraCalibration(;width,height,K)
end


# Camera extrinsic must be world in camera frame (cRw)
Base.@kwdef struct CameraExtrinsic{T <: Real}
  R::Rot_.RotMatrix{T} = id = one(Rot_.RotMatrix{3, Float64})
  t::Vector{T} = zeros(3)
end

Base.@kwdef struct CameraModelFull
  ci::CameraIntrinsic = CameraIntrinsic()
  ce::CameraExtrinsic = CameraExtrinsic()
end


export PinholeCamera

# ## From JuliaRobotics/Caesar.jl
# """
#     $TYPEDEF

# Pinhole Camera model is the most simplistic.

# Notes
# - https://en.wikipedia.org/wiki/Pinhole_camera
# - Standard Julia *[Images.jl](https://juliaimages.org/latest/)-frame* convention is, `size(img) <==> (i,j) <==> (height,width) <==> (y,x)`,
#   - Common *camera-frame* in computer vision and robotics, `(x,y) <==> (width,height) <==> (j,i)`,
#   - Using top left corner of image as `(0,0)` in all cases. 
#   - Direct comparison with [OpenCV convention](https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html) is:
#     - `(x,y) [CamXYZ] <==> (j,i) [Images.jl] <==> (u,v) [OpenCV]` -- look very carfully at `(u,v) <==> (j,i)` in *image-frame*
# - Always follow right hand rule for everything.

# DevNotes
# - https://en.wikipedia.org/wiki/Distortion_(optics)
# """
# struct PinholeCamera{R <: Real} <: AbstractCameraModel
#   K::SMatrix{3,3,R}
# end

## From JuliaRobotics/Caesar.jl
const PinholeCamera = (@warn("CameraModels.PinholeCamera is deprecated, use CamereModels.CameraCalibrationMutable instead."); CameraCalibration)
function PinholeCamera(
    K_::AbstractMatrix=[[510;0;320.0]';[0.0;510;240]';[0.0;0;1]']; # legacy constructor
    K::AbstractMatrix=[[fx;s;x0]';[0.0;fy;y0]';[0.0;0;1]'],        # consolidated matrix K
    width::Int=round(Int, K[1,3]*2), 
    height::Int=round(Int, K[2,3]*2),
  )
  #
  @warn "CameraModels.PinholeCamera is deprecated, use CamereModels.CameraCalibrationMutable instead."
  CameraCalibrationMutable(;width,height,K)
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


export Pinhole


## From yakir12/CameraModels.jl
const Pinhole = (@warn("CameraModels.Pinhole is deprecated, use CamereModels.CameraCalibration instead."); CameraCalibration)
function Pinhole(columns::Int,rows::Int,prinicipalpoint,focallength::Vector2 )
  @warn "CameraModels.Pinhole is deprecated, use CamereModels.CameraCalibration instead."
  CameraCalibration(;width=columns,height=rows,center=prinicipalpoint,focal=focallength)
end

function Base.getproperty(x::Pinhole, f::Symbol)
  if f == :columns
    getfield(x, :width)
  elseif f == :rows
    getfield(x, :height)
  elseif f == :prinicipalpoint
    SA[(getfield(x, :K)[1:2, 3])...]
  elseif f == :focallength
    K = getfield(x, :K)
    SA[K[1,1];K[2,2]] 
  else
    getfield(x, f)
  end
end

# ## From yakir12/CameraModels.jl
# struct Pinhole <: CameraModel
#   # note order of elements has been changed from original source so that inner constructor can be removed.
#   columns::Int
#   rows::Int
#   prinicipalpoint::PixelCoordinate # in pixels
#   focallength::Vector2             # in pixels
# end



#