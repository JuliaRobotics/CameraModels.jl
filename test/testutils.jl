using Test
using CameraModels


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