module CameraModels

using LinearAlgebra
using Manifolds
using DocStringExtensions
using StaticArrays
import Rotations as Rot_
import Base: getindex, getproperty


# exports
include("ExportAPI.jl")

# data types
include("entities/GeneralTypes.jl")
include("entities/CameraCalibration.jl")

include("services/CameraCalibration.jl")

# legacy implementations
include("Deprecated.jl")

# function logic
include("services/Prototypes.jl")
include("services/CameraServices.jl")
include("services/Utils.jl")




end # module
