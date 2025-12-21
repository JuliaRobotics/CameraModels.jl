using LinearAlgebra

function run_test_bench(
        model::C,
        pixel_accuracy::Float64 = 1.0e-5,
        ray_accuracy::Float64 = 1.0e-4
    ) where {C <: AbstractCameraModel}

    return @testset "Check basics and interface implementation for $(C)." begin

        forward = lookdirection(model)
        up = updirection(model)
        right = cross(forward, up)

        @testset "Test model axes" begin
            @test norm(forward) ≈ 1.0
            @test norm(up) ≈ 1.0
            @test norm(right) ≈ 1.0
        end

        pixelsize = sensorsize(model)

        # Get a pixel location somewhere near the image centre.
        image_centre = (pixelsize .+ 1) ./ 2
        slight_offset = 0.042 .* pixelsize
        some_pixel_location = image_centre + slight_offset

        # Get the ray that belongs to that pixel.
        # backproject returns a direction vector,
        # and we assume origin is (0,0,0) for now if not specified
        # actually Ray is defined as struct Ray origin::Point3 direction::Vector3
        dir = backproject(model, some_pixel_location)
        ray = Ray(Point3(0, 0, 0), dir)

        # Generate a 3D point along that ray.
        point = CameraModels.origin(ray) + 4.2 .* CameraModels.direction(ray)

        println(model)

        # Some models might not implement project.
        if canreproject(model)
            reprojection = project(model, point)
            @test_broken some_pixel_location[1] ≈ reprojection[1] atol = pixel_accuracy
            @test_broken some_pixel_location[2] ≈ reprojection[2] atol = pixel_accuracy
        else
            @info "project not implemented for $(C)."
        end

    end

end
