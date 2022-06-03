
principal_point = PixelCoordinate(55.4, 49.6)
focal_length = CameraModels.Vector2(61.2, 66.4)

model = Pinhole(100, 100, principal_point, focal_length)

run_test_bench(model)

@testset "Check specifics for Pinhole model." begin
    some_point = CameraModels.Point3(0, 1, 0)
    should_be_principal_point = point2pixel(model, some_point)
    @test principal_point ≈ should_be_principal_point

    ray = pixel2ray(model, principal_point)
    @test direction(ray) ≈ CameraModels.Vector3(0,1,0)
    @test origin(ray) ≈ CameraModels.Point3(0,0,0)

end

