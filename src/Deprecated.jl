


## consolidated types below


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
const PinholeCamera = (@warn("CameraModels.PinholeCamera is deprecated, use CamereModels.CameraCalibration instead."); CameraCalibration)
function PinholeCamera(
    K_::AbstractMatrix=[[510;0;320.0]';[0.0;510;240]';[0.0;0;1]']; # legacy constructor
    K::AbstractMatrix=[[fx;s;x0]';[0.0;fy;y0]';[0.0;0;1]'],        # consolidated matrix K
    width::Int=round(Int, K[1,3]*2), 
    height::Int=round(Int, K[2,3]*2),
  )
  #
  @warn "CameraModels.PinholeCamera is deprecated, use CamereModels.CameraCalibration instead."
  CameraCalibration(;width,height,K)
end



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
    view(getfield(x, :K), 1:2, 3)
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


export CameraModel # being replaced by AbstractCameraModel
# abstract type CameraModel end 


#