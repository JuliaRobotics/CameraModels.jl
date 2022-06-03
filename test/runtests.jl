using CameraModels
using Test
using StaticArrays

struct SomeTestModel <: CameraModels.CameraModel end

@testset "Test sensorsize using rows and columns." begin
    CameraModels.rows(m::SomeTestModel) = 11
    CameraModels.columns(m::SomeTestModel) = 22
    
    model = SomeTestModel()
    @test sensorsize(model) == SVector{2}(22,11)
end



include("CameraTestBench.jl")

include("Pinhole.jl")
