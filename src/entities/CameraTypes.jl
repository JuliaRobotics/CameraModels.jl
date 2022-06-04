
"""
    $TYPEDEF

Standard pinhole camera model with distortion parameters (aka camera intrinsics).  

Notes
- Image origin assumed as top-left.
- Keeping with Images.jl,
  - width of the image are matrix columns from left to right.
  - height of the image are matrix rows from top to bottom.
  - E.g. `mat[i,j] == img[h,w] == mat[h,w] == img[i,j]`
    - This is to leverage the unified Julia Arrays infrastructure, incl vectors, view, Static, CPU, GPU, etc.

Also see: (TODO: `ProjectiveCameraModel`)
"""
struct CameraCalibration{R <: Real} <: AbstractCameraModel
  """ numver of pixels from top to bottom """
  height::Int		# = 480
  """ number of pixels from left to right """
  width::Int		# = 640
  """ distortion coefficients up to fifth order """
  kc::Vector{Float64} # = zeros(5)
  """ 3x3 camera calibration matrix """
  K::SMatrix{3,3,R}   # = SMatrix{3,3}([[height;0.0;width/2]';[0.0;height;height/2]';[0.0;0;1]'] )
  """ inverse of a 3x3 camera calibration matrix """
  Ki::SMatrix{3,3,R}  # = inv(K)
end


CameraCalibration(;
  width::Int = 640,
  height::Int= 480,
  center::Union{<:AbstractVector,<:PixelCoordinate} = [width/2;height/2],
  focal::AbstractVector  = 1.1*[height; height], # just emperical default
  kc::AbstractVector{<:Real} = zeros(5),
  skew::Real = 0.0,
  K=SMatrix{3,3}([[focal[1];skew;center[1]]';[0.0;focal[2];center[2]]';[0.0;0;1]']),
) = CameraCalibration(height, width, kc, K, inv(K))



## FIXME consolidation necessary

const Pinhole = CameraCalibration
Pinhole(columns::Int,rows::Int,prinicipalpoint,focallength::Vector2 ) = CameraCalibration(;width=columns,height=rows,center=prinicipalpoint,focal=focallength)



## From JuliaRobotics/Caesar.jl
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


Base.@kwdef struct CameraIntrinsic
  K::Array{Float64,2}
end
CameraIntrinsic(;x0=320.0,y0=240.0,fx=510.0,fy=510.0,s=0.0) = CameraIntrinsic([[fx;s;x0]';[0.0;fy;y0]';[0.0;0;1]'])


# Camera extrinsic must be world in camera frame (cRw)
Base.@kwdef struct CameraExtrinsic{T <: Real}
  R::Rot_.RotMatrix{T} = id = one(Rot_.RotMatrix{3, Float64})
  t::Vector{T} = zeros(3)
end

Base.@kwdef struct CameraModelFull
  ci::CameraIntrinsic = CameraIntrinsic()
  ce::CameraExtrinsic = CameraExtrinsic()
end



"""
Data structure for a Camera model with parameters.
Use `CameraModel(width,height,fc,cc,skew,kc)` for easy construction.
"""
struct CameraModelandParameters <: AbstractCameraModel
    width::Int		# image width
    height::Int		# image height
    fc::Vector{Float64}	# focal length in x and y
    cc::Vector{Float64}	# camera center
    skew::Float64	    # skew value
    kc::Vector{Float64} # distortion coefficients up to fifth order
    K::Matrix{Float64} # 3x3 camera calibration matrix (Camera intrinsics)
    Ki::Matrix{Float64} # inverse of a 3x3 camera calibratio matrix
end
