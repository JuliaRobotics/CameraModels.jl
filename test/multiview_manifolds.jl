
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

cam = CameraModels.CameraCalibration()

obs1 = CameraModels.PixelIndex(240, 320)
obs2 = CameraModels.PixelIndex(240, 315)

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

function projectPointFrom(cam, c_H_w, w_Ph)
  c_Ph = c_H_w*w_Ph |> SVector{4}
  CameraModels.projectHomogeneous(cam,c_Ph)
end

function cameraResidual(cam, meas, M, w_T_c, w_Ph, κ=1000)
  pred = projectPointFrom(cam, inv(affine_matrix(M,w_T_c)), w_Ph)
  # experimental cost function to try force bad reprojects in front of the camera during optimization
  κ*(abs(pred.depth) - pred.depth)^2 + (meas[1]-pred[1])^2 + (meas[2]-pred[2])^2
end

function cost(w_Ph)
  cameraResidual(cam, obs1, M, w_T_c1, w_Ph) + cameraResidual(cam, obs2, M, w_T_c2, w_Ph)
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

@show w_P3 = w_Res.minimizer |> CameraModels.toNonhomogeneous
@test isapprox([10.56;0;0], w_P3; atol=1e-3)

##
end


##