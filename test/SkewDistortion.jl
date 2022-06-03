


## Tests from SensorFeatureTracking.jl

@testset "Test CameraSkewDistortion" begin

focald = 520.0
cu = 320.0
cv = 240.0
cam = CameraSkewDistortion(640,480,[focald, focald],[cv, cu], 0., [0])


focald = 520.0
cu = 0.0
cv = 0.0 # centred around zero for test data
cam = CameraSkewDistortion(640,480,[focald, focald],[cv, cu], 0., [0])


end

#