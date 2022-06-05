
## FIXME consolidation necessary

## =========================================================================================
## From yakir12/CameraModels.jl


origin(vector::Vector3) = origin3d
origin(ray::Ray) = vector.origin

lookdirection(cameramodel::CameraCalibrationT) = SVector{3}(0,1,0)
updirection(cameramodel::CameraCalibrationT) = SVector{3}(0,0,1)

width(cameramodel::CameraCalibrationT) = cameramodel.width
height(cameramodel::CameraCalibrationT) = cameramodel.height 

direction(vector::Vector3) = vector
direction(ray::Ray) = vector.direction

sensorsize(cameramodel::AbstractCameraModel) = SVector{2}(width(cameramodel), height(cameramodel))

"""
    canreproject(camera::CameraModel)

Confirms if point2pixel is implemented for this camera model.
"""
canreproject(camera::AbstractCameraModel) = true

"""
    point2pixel(model::Pinhole, pointincamera::$(Point3))

Return a transformation that converts real-world coordinates
to camera coordinates. This currently ignores any tangential 
distortion between the lens and the image plane.
"""
function point2pixel(model::CameraCalibrationT, pointincamera::Point3)
    column = model.prinicipalpoint[1] + model.focallength[1] * pointincamera[1] / pointincamera[2]
    row = model.prinicipalpoint[2] - model.focallength[2] * pointincamera[3] / pointincamera[2]
    return PixelCoordinate(column, row)
end


"""
    pixel2ray(model::Pinhole, pixelcoordinate::$(PixelCoordinate))

Return a transformation that converts real-world coordinates
to camera coordinates. This currently ignores any tangential 
distortion between the lens and the image plane.
"""
function pixel2ray(model::CameraCalibrationT, pixelcoordinate::PixelCoordinate)
    x = (pixelcoordinate[1] - model.prinicipalpoint[1]) / model.focallength[1]
    z = -(pixelcoordinate[2] - model.prinicipalpoint[2]) / model.focallength[2]
    return Vector3(x, 1, z)
end


## =========================================================================================
## From JuliaRobotics/Caesar.jl


f_w(pc::CameraCalibrationT) = pc.K[1,1]
f_h(pc::CameraCalibrationT) = pc.K[2,2]
shear(pc::CameraCalibrationT) = pc.K[1,2]
c_w(pc::CameraCalibrationT) = pc.K[1,3]
c_h(pc::CameraCalibrationT) = pc.K[2,3]

set_f_w!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1,1] = val)
set_f_h!(pc::CameraCalibrationMutable, val::Real) = (pc.K[2,2] = val)
set_shear!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1,2] = val)
set_c_w!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1,3] = val)
set_c_h!(pc::CameraCalibrationMutable, val::Real) = (pc.K[2,3] = val)



## ======================================================================================
## From JuliaRobotics/SensorFeatureTracking.jl

# function project!(ret::Vector{Float64}, ci::CameraIntrinsic, ce::CameraExtrinsic, pt::Vector{Float64})
#     res = ci.K*(ce.R*pt + ce.t)
#     ret[1:2] = res[1:2]./res[3]
#     nothing
#   end
#   project!(ret::Vector{Float64}, cm::CameraModelFull, pt::Vector{Float64}) = project!(ret, cm.ci, cm.ce, pt)
#   function project(cm::CameraModelFull, pt::Vector{Float64})
#     res = Vector{Float64}(2)
#     project!(res, cm, pt)
#     return res
#   end
  
#   # pinhole camera model
#   # (x, y)/f = (X, Y)/Z
#   function cameraResidual!(
#         res::Vector{Float64},
#         z::Vector{Float64},
#         ci::CameraIntrinsic,
#         ce::CameraExtrinsic,
#         pt::Vector{Float64}  )
#     # in place memory operations
#     project!(res, ci, ce, pt)
#     res[1:2] .*= -1.0
#     res[1:2] += z[1:2]
#     nothing
#   end



##