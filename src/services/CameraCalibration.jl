


CameraCalibration(;
  width::Int = 640,
  height::Int= 480,
  center::Union{<:AbstractVector{<:Real},<:PixelCoordinate} = [width/2;height/2],
  focal::AbstractVector{<:Real}  = 1.1*[height; height], # just emperical default
  kc::AbstractVector{<:Real} = SVector{5}(zeros(5)),
  skew::Real = 0.0,
  K::AbstractMatrix =[[focal[1];skew;center[1]]';[0.0;focal[2];center[2]]';[0.0;0;1]'],
) = CameraCalibration(height, width, kc, SMatrix{3,3}(K), SMatrix{3,3}(inv(K)) )

CameraCalibrationMutable(;
  width::Int = 640,
  height::Int= 480,
  center::Union{<:AbstractVector{<:Real},<:PixelCoordinate} = [width/2;height/2],
  focal::AbstractVector{<:Real}  = 1.1*[height; height], # just emperical default
  kc::AbstractVector{<:Real} = MVector{5}(zeros(5)),
  skew::Real = 0.0,
  K::AbstractMatrix =[[focal[1];skew;center[1]]';[0.0;focal[2];center[2]]';[0.0;0;1]'],
) = CameraCalibrationMutable(height, width, kc, MMatrix{3,3}(K), MMatrix{3,3}(inv(K)) )

"""
    $SIGNATURES

Constructor helper assuming you just have a camera image and need to start somewhere for a basic camera model.

Notes:
- Calibration will incorrect but hopefully in a distant ballpark to get the calibration process started.
- See [AprilTags.jl Calibration section](https://juliarobotics.org/AprilTags.jl/latest/#Camera-Calibration-1) for code and help. 
"""
function CameraCalibrationMutable(img::AbstractMatrix{T}) where T
  height, width = size(img)
  @warn "HERE" height width
  # emperical assumption usually seen for focal length
  f_w = 1.1*height
  f_h = f_w
  c_w, c_h = width/2, height/2
  @info "Assuming default CameraCalibrationMutable from image $(size(img)):" f_w f_h c_w c_h
  CameraCalibrationMutable(;width, height, focal=[f_w, f_h], center=[c_w, c_h])
end