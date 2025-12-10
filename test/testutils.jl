using Test
using CameraModels
import LieGroups
using LieGroups:
    SpecialEuclideanGroup,
    SpecialOrthogonalGroup,
    hat,
    exp,
    compose,
    LieAlgebra


@testset "Test intersect of line and plane" begin

    # Define plane
    floornorm = [0;0;1.0]
    floorcenter = [0;0;5.0]
    # Define ray
    raydir = [0;-1;-1.0]
    raypnt = [0; 0;10.0]

    pt = intersectLineToPlane3D(floornorm, floorcenter, raydir, raypnt)
    @test isapprox([0;-5;5.0], pt)

end


@testset "Test raytracing to plane" begin

    M = SpecialEuclideanGroup(3; variant = :right)
    Mr = SpecialOrthogonalGroup(3)
    R0 = [1 0 0; 0 1 0; 0 0 1.0]


    ## Camera setup
    f = 800.0        # pixels
    ci, cj = 360, 640  # assuming 720x1280 image
    # going from imaging array to camera frame
    c_H_a = [0 1 -cj; 1 0 -ci; 0 0 f] # camera matrix
    a_Forb = [360; 640; 1.0]
    l_nFL = [0; -0.05; 1.0]
    l_FL = [0; 0; -2.0]

    # local level to body to extrinsic transform
    l_T_b = ArrayPartition([0;0;0.0], R0)
    b_T_ex = ArrayPartition([0;0;0.0], exp(Mr, hat(LieAlgebra(Mr), [0;0.2;0.2])))
    l_T_ex = compose(M, l_T_b, b_T_ex)

    # Ray trace
    l_Forb = intersectRayToPlane(
        c_H_a,
        a_Forb,
        l_nFL,
        l_FL;
        l_T_ex
    )


    ## Place the body somewhere in the world
    w_T_b = ArrayPartition([0.0;0.0;2.0], LieGroups.exp(Mr, LieGroups.hat(LieAlgebra(Mr), [0;0;0.0])))
    # find feature points in the world frame
    # _w_Forb = MJL.affine_matrix(M, w_T_b)*[l_Forb; 1.]

end


@testset "radialDistortion! function test" begin

    test_camera = CameraCalibration(
        height = 720,
        width = 1280,
        kc = SVector(-0.12, 0.03, 0.0008, -0.0006, -0.004),
        K = SMatrix{3, 3}(
            [
                800.0 0.0   360.0;
                0.0   800.0 640.0;
                0.0   0.0   1.0
            ]
        )
    )


    src_mat = rand(Float32, 720, 1280)
    dst_mat = similar(src_mat)

    CameraModels.radialDistortion!(test_camera, dst_mat, src_mat)

end
