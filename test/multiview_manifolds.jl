using Test
import CameraModels
using Optim, LieGroups
using StaticArrays
using LieGroups: SpecialEuclideanProductPoint

M = SpecialEuclideanGroup(3; variant = :right)

@testset "Multiview optimization of point in front of 2 cameras" begin

cam = CameraModels.CameraCalibration()

obs1 = CameraModels.PixelIndex(320, 240)
obs2 = CameraModels.PixelIndex(315, 240)

w_T_c1 = ArrayPartition([0; 0  ;0.],[0 0 1; -1 0 0; 0 -1 0.])
w_T_c2 = ArrayPartition([0;-0.1;0.],[0 0 1; -1 0 0; 0 -1 0.])


function projectPointFrom(cam, c_H_w, w_Ph)
  c_Ph = c_H_w*w_Ph |> SVector{4}
  CameraModels.projectHomogeneous(cam,c_Ph)
end

function cameraResidual(cam, meas, M, w_T_c, w_Ph, κ=1000)
  pred = projectPointFrom(cam, inv(convert(AbstractMatrix, SpecialEuclideanProductPoint(w_T_c))), w_Ph)
  # experimental cost function to try force bad reprojects in front of the camera during optimization
  κ*(abs(pred.depth) - pred.depth)^2 + (meas[1]-pred[1])^2 + (meas[2]-pred[2])^2
end

function cost(w_P)
  w_Ph = if length(w_P) == 3
    SVector(w_P[1], w_P[2], w_P[3], 1.0)
  else
    w_P
  end
  cameraResidual(cam, obs1, M, w_T_c1, w_Ph) + cameraResidual(cam, obs2, M, w_T_c2, w_Ph)
end


w_Ph = SVector(10.,0.,0.,1.)

cost(w_Ph)

cost(SVector(0.1,0.,0.,1.))
cost(SVector(0.5,0.,0.,1.))


w_Res = Optim.optimize(
  cost, 
  [1.0;0.0;0.0], 
  LBFGS(),
)


@show w_Res.minimizer
@show w_P3 = w_Res.minimizer
@test isapprox([10.56;0;0], w_P3; atol=1e-3)

end
