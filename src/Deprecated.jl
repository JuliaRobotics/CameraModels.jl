
## ================================================================================================
## consolidated types from various repos in Julia ecosystem
## ================================================================================================

@deprecate project(cm::CameraModelFull, pt::AbstractVector{<:Real}) project(cm.ci,pt) # drops extrinsics

# function project(
#   cm::CameraModelFull, 
#   pt::AbstractVector{<:Real}
# )
#   res = Vector{Float64}(2)
#   project!(res, cm, pt)
#   return res
# end

"""
    CameraCalibrationT

A Union type for users to implement against both `struct`` and `mutable struct` definitions of `CameraCalibration`
"""
CameraCalibrationT = Union{<:CameraCalibration, <:CameraCalibrationMutable}

@deprecate CameraExtrinsic(R::AbstractMatrix=[1 0 0; 0 1 0; 0 0 1.], t::AbstractVector=[0,0,0.]) ArrayPartition(SVector(t...),SMatrix(R))

# Camera extrinsic must be world in camera frame (cRw)
# Base.@kwdef struct CameraExtrinsic{T <: Real}
#   R::SMatrix{3,3,T,9} = one(Rot_.RotMatrix{3, Float64}).mat
#   t::SVector{3,T} = SVector(0,0,0.)
# end


@deprecate PixelCoordinate(row,col) PixelIndex(row,col)

# CameraCalibration(
#   height::Int= 480,
#   width::Int = 640,
#   center::Union{<:AbstractVector{<:Real},<:PixelCoordinate} = [width/2;height/2],
#   focal::AbstractVector{<:Real}  = 1.1*[height; height], # just emperical default
#   kc::AbstractVector{<:Real} = SVector{5}(zeros(5)),
#   skew::Real = 0.0,
#   K::AbstractMatrix =[[focal[1];skew;center[1]]';[0.0;focal[2];center[2]]';[0.0;0;1]'],
# ) = CameraCalibration(height, width, kc, SMatrix{3,3}(K), SMatrix{3,3}(inv(K)) )
# CameraCalibrationMutable(;
#   width::Int = 640,
#   height::Int= 480,
#   center::Union{<:AbstractVector{<:Real},<:PixelCoordinate} = [width/2;height/2],
#   focal::AbstractVector{<:Real}  = 1.1*[height; height], # just emperical default
#   kc::AbstractVector{<:Real} = MVector{5}(zeros(5)),
#   skew::Real = 0.0,
#   K::AbstractMatrix =[[focal[1];skew;center[1]]';[0.0;focal[2];center[2]]';[0.0;0;1]'],
# ) = CameraCalibrationMutable(height, width, kc, MMatrix{3,3}(K), MMatrix{3,3}(inv(K)) )



@deprecate point2pixel(model, pt) project(model, pt[[1;3;2]])
@deprecate pixel2ray(model, px) backproject(model, px)[[1;3;2]]


# """
#     point2pixel(model::Pinhole, pointincamera::$(Point3))

# Return a transformation that converts real-world coordinates
# to camera coordinates. This currently ignores any tangential 
# distortion between the lens and the image plane.
# """
# function point2pixel(model::CameraCalibrationT, pointincamera::Point3)
#     column = model.prinicipalpoint[1] + model.focallength[1] * pointincamera[1] / pointincamera[2]
#     row = model.prinicipalpoint[2] - model.focallength[2] * pointincamera[3] / pointincamera[2]
#     return PixelCoordinate(column, row)
# end


# """
#     pixel2ray(model::Pinhole, pixelcoordinate::$(PixelCoordinate))

# Return a transformation that converts real-world coordinates
# to camera coordinates. This currently ignores any tangential 
# distortion between the lens and the image plane.
# """
# function pixel2ray(model::CameraCalibrationT, pixelcoordinate::PixelCoordinate)
#     x = (pixelcoordinate[1] - model.prinicipalpoint[1]) / model.focallength[1]
#     z = -(pixelcoordinate[2] - model.prinicipalpoint[2]) / model.focallength[2]
#     return Vector3(x, 1, z)
# end


export CameraModel # being replaced by AbstractCameraModel
CameraModel = (@warn("CameraModels.CameraModel is deprecated, use CameraModels.AbstractCameraModel instead");AbstractCameraModel)
# abstract type CameraModel end 

@warn "RadialDistortion is deprecated, use CameraCalibration instead"
# Base.@kwdef struct RadialDistortion{N, R <: Real, K <: AbstractVector}
#   Ki::SVector{N,R} = SVector(0.0) # 
#   center::SVector{2,R} = SVector{2,R}(0.0,0.0) # SVector{2,R} # [h,w]
#   # _radius2::Matrix{R} # perhaps SizedArray{R,2} or StaticArray{R,2} or GPUArray{R,2} depending on performance
# end


@deprecate columns(w...;kw...) width(w...;kw...)
@deprecate rows(w...;kw...) height(w...;kw...)

# sensorsize(cameramodel::CameraModel) = SVector{2}(width(cameramodel), height(cameramodel))


export CameraModelandParameters
# const CameraModelandParameters = (@warn("CameraModels.CameraModelandParameters is deprecated, use CameraModels.CameraCalibration instead.");CameraCalibration)

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
  @warn("CameraModels.CameraModelandParameters is deprecated, use CameraModels.CameraCalibration instead.")
  CameraCalibration(;width,height,K=SMatrix{3,3}(K))
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
# const PinholeCamera = (@warn("CameraModels.PinholeCamera is deprecated, use CameraModels.CameraCalibrationMutable instead."); CameraCalibration)

function PinholeCamera(
  K_::AbstractMatrix=[[510;0;320.0]';[0.0;510;240]';[0.0;0;1]']; # legacy constructor
  width::Int=round(Int, K_[1,3]*2), 
  height::Int=round(Int, K_[2,3]*2),
  f_w::Real=K_[1,1],
  f_h::Real=K_[2,2],
  c_w::Real=K_[1,3],
  c_h::Real=K_[2,3],
  shear::Real=K_[1,2],
  K::AbstractMatrix=[[f_w;shear;c_w]';[0.0;f_h;c_h]';[0.0;0;1]'],        # consolidated matrix K
)
  #
  @warn "CameraModels.PinholeCamera is deprecated, use CameraModels.CameraCalibrationMutable instead."
  if 3 < size(K_,1)
    @warn "PinholeCamera(arg), 3 < size(arg,1), assuming legacy constructor as img as input argument."
    return CameraCalibrationMutable(K_) # as though img=K_
  end
  CameraCalibrationMutable(;width,height,K=MMatrix{3,3}(K))
end

# @deprecate PinholeCamera(img::AbstractMatrix) CameraCalibrationMutable(img)

# function PinholeCamera(img::AbstractMatrix{T}) where T
#   f_w, c_w, c_h = size(img, 1), size(img, 2)/2, size(img, 1)/2
#   f_h = f_w
#   @info "Assuming default PinholeCamera from image $(size(img)):" f_w f_h c_w c_h
#   PinholeCamera(;f_w, f_h, c_w, c_h)
# end

@deprecate c_w(w...) pp_w(w...)
@deprecate c_h(w...) pp_h(w...)
# f_w(pc::Union{CameraCalibration,CameraCalibrationMutable}) = pc.K[1,1]
# f_h(pc::Union{CameraCalibration,CameraCalibrationMutable}) = pc.K[2,2]
# shear(pc::Union{CameraCalibration,CameraCalibrationMutable}) = pc.K[1,2]
# pp_w(pc::Union{CameraCalibration,CameraCalibrationMutable}) = pc.K[1,3]
# pp_h(pc::Union{CameraCalibration,CameraCalibrationMutable}) = pc.K[2,3]

# set_f_w!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1,1] = val)
# set_f_h!(pc::CameraCalibrationMutable, val::Real) = (pc.K[2,2] = val)
# set_shear!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1,2] = val)
# set_pp_w!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1,3] = val)
# set_pp_h!(pc::CameraCalibrationMutable, val::Real) = (pc.K[2,3] = val)


export Pinhole


## From yakir12/CameraModels.jl
# const Pinhole = (@warn("CameraModels.Pinhole is deprecated, use CameraModels.CameraCalibration instead."); CameraCalibration)
function Pinhole(columns::Int,rows::Int,prinicipalpoint,focallength::Vector2 )
  @warn "CameraModels.Pinhole is deprecated, use CameraModels.CameraCalibration instead."
  f_w,f_h = focallength[1], focallength[2]
  c_w,c_h = prinicipalpoint[1], prinicipalpoint[2]
  K = SMatrix{3,3}([[f_w;0.0;c_w]';[0.0;f_h;c_h]';[0.0;0;1.]'] )
  CameraCalibration(;
    height=rows,
    width=columns,
    K
  )
end

function CameraIntrinsic(
  K_::AbstractMatrix=[[510;0;320.0]';[0.0;510;240]';[0.0;0;1]']; # legacy constructor
  x0=320.0,y0=240.0,fx=510.0,fy=510.0,s=0.0,                     # legacy function support
  K::AbstractMatrix=[[fx;s;x0]';[0.0;fy;y0]';[0.0;0;1]'],        # consolidated matrix K
  width::Int=round(Int, K[1,3]*2), 
  height::Int=round(Int, K[2,3]*2),
)
  #
  @warn "CameraModels.CameraIntrinsic is deprecated, use CameraModels.CameraCalibration instead."
  CameraCalibration(;width,height,K)
end
export CameraIntrinsic

# Base.@kwdef struct CameraIntrinsic
#   K::Array{Float64,2}
# end
# CameraIntrinsic(;x0=320.0,y0=240.0,fx=510.0,fy=510.0,s=0.0) = CameraIntrinsic([[fx;s;x0]';[0.0;fy;y0]';[0.0;0;1]'])

## From JuliaRobotics/Caesar.jl
# const CameraIntrinsic = (@warn("CameraModels.CameraIntrinsic is deprecated, use CameraModels.CameraCalibration instead.");CameraCalibration)


function Base.getproperty(
  x::CameraCalibration, # Union{<:Pinhole, CameraModelandParameters},
  f::Symbol,
)
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
# function Base.getproperty(x::CameraModelandParameters, f::Symbol)
#   if f == :skew
#     getfield(x, :K)[1,2]
#   elseif f == :columns
#     getfield(x, :width)
#   elseif f == :rows
#     getfield(x, :height)
#   elseif f == :prinicipalpoint || f == :cc
#     SA[(getfield(x, :K)[1:2, 3])...]
#   elseif f == :focallength || f == :fc
#     K = getfield(x, :K)
#     SA[K[1,1];K[2,2]] 
#   else
#     getfield(x, f)
#   end
# end

# ## From yakir12/CameraModels.jl
# struct Pinhole <: CameraModel
#   # note order of elements has been changed from original source so that inner constructor can be removed.
#   columns::Int
#   rows::Int
#   prinicipalpoint::PixelCoordinate # in pixels
#   focallength::Vector2             # in pixels
# end





#