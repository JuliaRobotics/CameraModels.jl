


CameraCalibration(;
  width::Int = 640,
  height::Int= 480,
  center::Union{<:AbstractVector{<:Real},<:PixelCoordinate} = [width/2;height/2],
  focal::AbstractVector{<:Real}  = 1.1*[height; height], # just emperical default
  kc::AbstractVector{<:Real} = zeros(5),
  skew::Real = 0.0,
  K::AbstractMatrix =[[focal[1];skew;center[1]]';[0.0;focal[2];center[2]]';[0.0;0;1]'],
) = CameraCalibration(height, width, kc, SMatrix{3,3}(K), SMatrix{3,3}(inv(K)) )

