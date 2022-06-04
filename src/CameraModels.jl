module CameraModels

using DocStringExtensions
using StaticArrays
import Rotations as Rot_



# exports
include("ExportAPI.jl")

# data types
include("entities/GeneralTypes.jl")
include("entities/CameraTypes.jl")
include("entities/RadialDistortion.jl")

# function logic
include("services/Prototypes.jl")
include("services/CameraServices.jl")
include("services/RadialDistortion.jl")
include("services/Utils.jl")

# legacy implementations
include("Deprecated.jl")



end # module
