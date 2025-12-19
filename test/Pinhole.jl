
@testset "Check CameraCalibrationMutable basics" begin

    img = zeros(80,120) # a landscape image size (height, width) as per Images.jl
    ccm = CameraCalibrationMutable(img)

    @test height(ccm) == 80
    @test width(ccm) == 120
end
