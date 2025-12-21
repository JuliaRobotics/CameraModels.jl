# Abstract type
abstract type AbstractCameraModel end


struct PixelIndex{VALID, T <: Real}
    row::T
    col::T
    depth::T
end

PixelIndex(
    u::T,
    v::T;
    valid::Bool = true,
    depth = zero(T)
) where {T <: Real} = PixelIndex{valid, T}(u, v, depth)

Base.getindex(p::PixelIndex, i::Int) =
    i == 1 ? p.row :
    i == 2 ? p.col :
    i == 3 ? p.depth :
    throw(DomainError(i, "Camera only has rows, columns and depth"))


const Vector2 = Vec{2, Float64}
const Point3 = Point{3, Float64}
const Vector3 = Vec{3, Float64}


origin3d = Point3(0, 0, 0)

struct Ray
    origin::Point3
    direction::Vector3
end
