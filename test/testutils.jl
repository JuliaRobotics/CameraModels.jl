using Test
using CameraModels
using Manifolds


@testset "Test intersect of line and plane" begin

# Define plane
floornorm = [0;0;1.]
floorcenter  = [0;0;5.]
# Define ray
raydir = [0;-1;-1.]
raypnt = [0; 0;10.]

pt = intersectLineToPlane3D(floornorm, floorcenter, raydir, raypnt)
@test isapprox([0;-5;5.], pt)

end


@testset "Test raytracing to plane" begin

M = SpecialEuclidean(3)
Mr = M.manifold[2]
R0 = [1 0 0; 0 1 0; 0 0 1.]


## Camera setup
f = 800.0        # pixels
ci,cj = 360,640  # assuming 720x1280 image
# going from imaging array to camera frame
c_H_a = [0 1 -cj; 1 0 -ci; 0 0 f] # camera matrix
a_Forb = [360; 640; 1.0]
l_nFL = [0; -0.05; 1.]
l_FL = [0; 0; -2.]

# local level to body to extrinsic transform 
l_T_b = ArrayPartition([0;0;0.], R0)
b_T_ex = ArrayPartition([0;0;0.], exp_lie(Mr, hat(Mr, R0, [0;0.2;0.2])))
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
w_T_b = ArrayPartition([0.;0.;2.], exp_lie(Mr, hat(Mr, R0, [0;0;0.])))
# find feature points in the world frame
_w_Forb = affine_matrix(M, w_T_b)*[l_Forb; 1.]

end