

struct PixelIndex{VALID, T <: Real}
    row::T
    col::T
    depth::T
end
PixelIndex(u::T, v::T; valid::Bool=true, depth = T(0)) where {T <: Real} = PixelIndex{valid,T}(u, v, depth)

function Base.getindex(p::PixelIndex,i::Int)
    if i === 1
        p.row
    elseif i === 2  
        p.col
    elseif i === 3
        p.depth
    else
        DomainError("Camera only has rows and columns, cannot index to $i")
    end
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