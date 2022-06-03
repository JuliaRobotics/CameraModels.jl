

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
  # cd::CameraDistortion
end
