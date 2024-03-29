
@testset "Check CameraCalibrationMutable basics" begin

    img = zeros(80,120) # a landscape image size (height, width) as per Images.jl
    ccm = CameraCalibrationMutable(img)

    @test height(ccm) == 80
    @test width(ccm) == 120
end


principal_point = PixelIndex(55.4, 49.6)
focal_length = CameraModels.Vector2(61.2, 66.4)


model = Pinhole(100, 100, principal_point, focal_length)


run_test_bench(model)

@testset "Check Legacy Pinhole model." begin
    @show some_point = CameraModels.Point3(0, 1, 0)
    @show should_be_principal_point = point2pixel(model, some_point)
    @test_broken principal_point[1] ≈ should_be_principal_point[1]
    @test_broken principal_point[2] ≈ should_be_principal_point[2]

    ray = pixel2ray(model, principal_point)
    @test CameraModels.direction(ray) ≈ CameraModels.Vector3(0,1,0)
    @test CameraModels.origin(ray) ≈ CameraModels.Point3(0,0,0)

end

@testset "Check Legacy PinholeCamera" begin
    
    PinholeCamera()

    @info "Legacy test on PinholeCamera from generic image"
    img = zeros(80,120) # a landscape image size (height, width) as per Images.jl
    ccm = PinholeCamera(img)

    @test height(ccm) == 80
    @test width(ccm) == 120

end

#