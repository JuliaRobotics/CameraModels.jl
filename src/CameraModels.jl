module CameraModels

using DocStringExtensions
using StaticArrays
import Rotations as Rot_



# exports
include("ExportAPI.jl")

# data types
include("entities/GeneralTypes.jl")
include("entities/IntrinsicExtrinsic.jl")
include("entities/Pinhole.jl")
include("entities/RadialDistortion.jl")

# function logic
include("services/Prototypes.jl")
include("services/Pinhole.jl")
include("services/RadialDistortion.jl")



end # module
