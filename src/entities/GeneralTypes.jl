struct PixelIndex{VALID, T <: Real}
    row::T
    col::T
    depth::T
end
PixelIndex(u::T, v::T; valid::Bool = true, depth = T(0)) where {T <: Real} = PixelIndex{valid, T}(u, v, depth)

Base.getindex(p::PixelIndex, i::Int) =
    i == 1 ? p.row :
    i == 2 ? p.col :
    i == 3 ? p.depth :
    throw(DomainError(i, "Camera only has rows, columns and depth"))


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
