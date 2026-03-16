module CameraModels

using LinearAlgebra
using LieGroups
using DocStringExtensions
using SparseArrays
using StaticArrays
using FixedPointNumbers
using StatsBase
using ImageCore: colorview, RGB
import Rotations as Rot_
import Base: getindex, getproperty, show
using RecursiveArrayTools: ArrayPartition
using LoopVectorization: @tturbo

# exports
include("ExportAPI.jl")

# data types
include("entities/GeneralTypes.jl")
include("entities/CameraCalibration.jl")

include("services/CameraCalibration.jl")
include("services/RadianceCorrection.jl") # EXPERIMENTAL, not public yet

# legacy implementations
include("Deprecated.jl")

# function logic
include("services/Prototypes.jl")
include("services/CameraServices.jl")
include("services/Utils.jl")


end # module
