

struct PixelIndex{T <: Real}
    row::T
    col::T
end


const Vector2 = SVector{2, Float64}
const Point3 = SVector{3, Float64}
const Vector3 = SVector{3, Float64}

# Abstract type
abstract type AbstractCameraModel end

origin3d = zeros(Point3)

struct Ray
    origin::Point3
    direction::Vector3
end