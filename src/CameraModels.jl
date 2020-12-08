module CameraModels

export AbstractCameraModel

abstract type AbstractCameraModel end

struct PinholeCamera <: AbstractCameraModel
    K::SArray{Float64,2}
end

end # module
