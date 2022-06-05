
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

PinholeCamera(; f_w::Real=300,
                f_h::Real=f_w,
                c_w::R=320.0,
                c_h::Real=240,
                shear::Real=0) where {R <: Real} = SMatrix{3,3,R}([f_w shear c_w;
                                                                    0   f_h  c_h;
                                                                    0     0    1])
#

"""
    $SIGNATURES

Constructor helper assuming you just have a camera image and need to start somewhere for a basic camera model.

Notes:
- Calibration will incorrect but hopefully in a distant ballpark to get the calibration process started.
- See [AprilTags.jl Calibration section](https://juliarobotics.org/AprilTags.jl/latest/#Camera-Calibration-1) for code and help. 
"""
function PinholeCamera(img::AbstractArray{T,2}) where T
  f_w, c_w, c_h = size(img, 1), size(img, 2)/2, size(img, 1)/2
  f_h = f_w
  @info "Assuming default PinholeCamera from image $(size(img)):" f_w f_h c_w c_h
  PinholeCamera(f_w=f_w, f_h=f_h, c_w=c_w, c_h=c_h)
end




## ======================================================================================
## From JuliaRobotics/SensorFeatureTracking.jl

function project!(ret::Vector{Float64}, ci::CameraIntrinsic, ce::CameraExtrinsic, pt::Vector{Float64})
    res = ci.K*(ce.R*pt + ce.t)
    ret[1:2] = res[1:2]./res[3]
    nothing
  end
  project!(ret::Vector{Float64}, cm::CameraModelFull, pt::Vector{Float64}) = project!(ret, cm.ci, cm.ce, pt)
  function project(cm::CameraModelFull, pt::Vector{Float64})
    res = Vector{Float64}(2)
    project!(res, cm, pt)
    return res
  end
  
  # pinhole camera model
  # (x, y)/f = (X, Y)/Z
  function cameraResidual!(
        res::Vector{Float64},
        z::Vector{Float64},
        ci::CameraIntrinsic,
        ce::CameraExtrinsic,
        pt::Vector{Float64}  )
    # in place memory operations
    project!(res, ci, ce, pt)
    res[1:2] .*= -1.0
    res[1:2] += z[1:2]
    nothing
  end



##