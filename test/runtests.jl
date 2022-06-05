using CameraModels
using Test
using StaticArrays

import CameraModels: height, width

struct SomeTestModel <: CameraModels.AbstractCameraModel end

# supposed to override through dispatch???
CameraModels.height(m::SomeTestModel) = 11
CameraModels.width(m::SomeTestModel) = 22

@testset "Test sensorsize using rows and columns." begin
    
    model = SomeTestModel()
    @test sensorsize(model) == SVector{2}(22,11)
end



include("CameraTestBench.jl")
include("Pinhole.jl")
include("SkewDistortion.jl")

