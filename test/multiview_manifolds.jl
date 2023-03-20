
#

using Test
# using Revise
import CameraModels
using Optim, Manifolds
using StaticArrays
# using ManifoldDiff
# import FiniteDifferences as FD

##

M = SpecialEuclidean(3)

##
@testset "Multiview optimization of point in front of 2 cameras" begin
##

cam = CameraCalibration()

obs1 = PixelIndex(240, 320)
obs2 = PixelIndex(240, 315)

w_T_c1 = ArrayPartition([0; 0  ;0.],[0 0 1; -1 0 0; 0 -1 0.])
w_T_c2 = ArrayPartition([0;-0.1;0.],[0 0 1; -1 0 0; 0 -1 0.])

# w
# [
# 0 -0.1 0
# ]
# =
# w
# [
#  0  0  1
# -1  0  0
#  0 -1  0
# ]
# c
# [
# 0.1 0 0
# ]

##

function cost(w_Ph)
  c1_Ph = affine_matrix(M, inv(M,w_T_c1))*w_Ph |> SVector{4}
  pre1 = CameraModels.projectHomogeneous(cam,c1_Ph)

  c2_Ph = affine_matrix(M, inv(M,w_T_c2))*w_Ph |> SVector{4}
  pre2 = CameraModels.projectHomogeneous(cam,c2_Ph)
  
  (obs1[1]-pre1[1])^2 + (obs1[2]-pre1[2])^2 + (obs2[1]-pre2[1])^2 + (obs2[2]-pre2[2])^2
end

##

w_Ph = SVector(10.,0.,0.,1.)

cost(w_Ph)

cost(SVector(0.1,0.,0.,1.))
cost(SVector(0.5,0.,0.,1.))

##

w_Res = Optim.optimize(
  cost, 
  [1;0;0;1.], 
  LBFGS(),
  # ParticleSwarm(; lower = [0.,-1.,-1.,0.],
  #               upper = [99999.;1;1;9999],
  #               n_particles = 10),
  # Optim.Options(g_tol = 1e-12,
  #               iterations = 100,
  #               store_trace = false,
  #               show_trace = true);
  # autodiff=:forward
)


@show w_Res.minimizer

##


@show w_Res.minimizer |> toNonhomogeneous

@test isapprox([10.56;0;0], toNonhomogeneous(w_Res.minimizer); atol=1e-3)

##
end


##