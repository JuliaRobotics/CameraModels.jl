"""
    origin(ray)

Return the origin of the ray as a `Vector3`.
"""
origin(vector::Union{<:AbstractVector{<:Real}, <:Vector3}) = origin3d
origin(ray::Ray) = ray.origin

"""
    lookdirection(camera::AbstractCameraModel)

Return the lookdirection of this camera model.
"""
lookdirection(cameramodel::AbstractCameraModel) = Vector3(0, 1, 0)

"""
    updirection(camera::AbstractCameraModel)

Return the updirection of this camera model.
"""
updirection(cameramodel::AbstractCameraModel) = Vector3(0, 0, 1)

"""
    width(model::AbstractCameraModel)

Returns the width (columns) of the camera sensor.
"""
width(cameramodel::AbstractCameraModel) = cameramodel.width

"""
    height(model::AbstractCameraModel)

Returns the height (rows) of the camera sensor.
"""
height(cameramodel::AbstractCameraModel) = cameramodel.height

"""
    direction(ray)

Return the direction of the ray as a `Vector3`.
"""
direction(vector::Union{<:AbstractVector{<:Real}, <:Vector3}) = vector
direction(ray::Ray) = ray.direction

"""
    sensorsize(model::AbstractCameraModel)

Return the size of the camera sensor. By default calling out to width(model) and height(model) to build a Vec{2}

`sensorsize(cameramodel::AbstractCameraModel) = Vec{2}(width(cameramodel), height(cameramodel))`
"""
sensorsize(cameramodel::AbstractCameraModel) = Vec{2}(width(cameramodel), height(cameramodel))


"""
    f_w(pc::AbstractCameraModel)

Return the focal length in the width direction.
"""
f_w(pc::AbstractCameraModel) = pc.K[1, 1]

"""
    f_h(pc::AbstractCameraModel)

Return the focal length in the height direction.
"""
f_h(pc::AbstractCameraModel) = pc.K[2, 2]

"""
    shear(pc::AbstractCameraModel)

Return the shear parameter of the camera.
"""
shear(pc::AbstractCameraModel) = pc.K[1, 2]

"""
    pp_w(pc::AbstractCameraModel)

Return the principal point in the width direction.
"""
pp_w(pc::AbstractCameraModel) = pc.K[1, 3]

"""
    pp_h(pc::AbstractCameraModel)

Return the principal point in the height direction.
"""
pp_h(pc::AbstractCameraModel) = pc.K[2, 3]

"""
    set_f_w!(pc::CameraCalibrationMutable, val::Real)

Set the focal length in the width direction.
"""
set_f_w!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1, 1] = val)

"""
    set_f_h!(pc::CameraCalibrationMutable, val::Real)

Set the focal length in the height direction.
"""
set_f_h!(pc::CameraCalibrationMutable, val::Real) = (pc.K[2, 2] = val)

"""
    set_shear!(pc::CameraCalibrationMutable, val::Real)

Set the shear parameter of the camera.
"""
set_shear!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1, 2] = val)

"""
    set_pp_w!(pc::CameraCalibrationMutable, val::Real)

Set the principal point in the width direction.
"""
set_pp_w!(pc::CameraCalibrationMutable, val::Real) = (pc.K[1, 3] = val)

"""
    set_pp_h!(pc::CameraCalibrationMutable, val::Real)

Set the principal point in the height direction.
"""
set_pp_h!(pc::CameraCalibrationMutable, val::Real) = (pc.K[2, 3] = val)

"""
    canreproject(camera::AbstractCameraModel)

Confirms if project is implemented for this camera model.
"""
canreproject(camera::AbstractCameraModel) = true


## Computational functions


## =========================================================================================
## FROM SCENE IN FRONT OF CAMERA TO IMAGE -- I.E. PROJECT
## =========================================================================================


function undistortPoint(cam::CameraCalibration, xy, iter_num = 3)
    k1, k2, p1, p2, k3 = cam.kc[1:5]
    fx, fy = f_w(cam), f_h(cam) # cam.K[1, 1], cam.K[2, 2]
    cx, cy = pp_w(cam), pp_h(cam) # cam.K[1:2, 3]
    x, y = xy[1], xy[2]
    x = (x - cx) / fx
    x0 = x
    y = (y - cy) / fy
    y0 = y
    for _ in 1:iter_num
        r2 = x^2 + y^2
        k_inv = 1 / (1 + k1 * r2 + k2 * r2^2 + k3 * r2^3)
        delta_x = 2 * p1 * x * y + p2 * (r2 + 2 * x^2)
        delta_y = p1 * (r2 + 2 * y^2) + 2 * p2 * x * y
        x = (x0 - delta_x) * k_inv
        y = (y0 - delta_y) * k_inv
    end
    return Point(x * fx + cx, y * fy + cy)
end


## =========================================================================================
## FROM SCENE IN FRONT OF CAMERA TO IMAGE -- I.E. PROJECT
## =========================================================================================

function project!(
        ret::AbstractVector{<:Real},
        ci::CameraCalibration,
        c_T_r::ArrayPartition,
        r_P::AbstractVector{<:Real}
    )
    res = ci.K * (c_T_r.x[2] * r_P + c_T_r.x[1])
    ret[1:2] ./= res[3]
    return PixelIndex(ret[1], ret[2])
end

"""
    $SIGNATURES

Project a world scene onto an image.

Returns the pixel location onto which the 3D coordinate `r_P` is projected.
This currently ignores any tangential distortion between the lens and the image plane.

Notes
- `r_P` is a point in reference frame transformed the camera's reference frame:
  - `c_P = c_T_r * r_P`

Also see: [`backproject`](@ref)
"""
function project(
        model::AbstractCameraModel,
        r_P::Union{<:AbstractVector{<:Real}, <:Point3};
        c_T_r = ArrayPartition(Vector3(0.0, 0.0, 0.0), Mat{3,3}(1.0*I(3)))
    )
    ret = MVector(0.0, 0.0)
    return project!(ret, model, c_T_r, r_P)
end


"""
    $SIGNATURES

Project a 3D point in homogeneous coordinates `c_Ph` (camera frame) to a `PixelIndex`.
"""
function projectHomogeneous(
    cam::AbstractCameraModel,
    c_Ph::AbstractVector{<:Real}
)
    x, y, z, w = c_Ph
    
    # Project to image plane
    inv_z = 1 / z
    col = x * f_w(cam) * inv_z + pp_w(cam)
    row = y * f_h(cam) * inv_z + pp_h(cam)
    
    # Depth and validity (point must be in front of camera)
    depth = z / w
    valid = (w == 0 && z > 0) || depth > 0
    
    return PixelIndex(col, row; depth, valid)
end


## =========================================================================================
## FROM IMAGE TO SCENE IN FRONT OF CAMERA -- I.E. BACKPROJECT
## =========================================================================================


"""
    $SIGNATURES

Backproject from an image into a world scene.

Returns the ray in space (direction vector) corresponding to this `pixelIndex`.
This currently ignores any tangential distortion between the lens and the image plane.

Also see: [`project`](@ref)
"""
function backproject(
        model::AbstractCameraModel,
        px_coord::Union{<:AbstractVector{<:Real}, <:PixelIndex}
    )
    #
    col = (px_coord[1] - pp_w(model)) / f_w(model)
    row = -(px_coord[2] - pp_h(model)) / f_h(model)
    return Vector3(col, row, 1)
end


# # camera measurements (u,v), (u2,v)
# lx = (u-center[1])*baseline
# ly = (v-center[2])*baseline
# lz = _f*baseline
# lw = u - u2
# lw<0 ? @warn("backprojecting negative disparity\n") : nothing
# # homogeneous point coords
# return (lz, lx, ly, lw)


## =========================================================================================
## RESIDUAL FUNCTION FOR OPTIMIZATION OR LOSS
## =========================================================================================


# pinhole camera model
# (x, y)/f = (X, Y)/Z
function cameraResidual!(
        res::AbstractVector{<:Real},
        z::AbstractVector{<:Real},
        ci::CameraCalibration,
        ce::ArrayPartition,
        pt::Union{PixelIndex, <:AbstractVector{<:Real}},
    )
    # in place memory operations
    project!(res, ci, ce, pt)
    res[1:2] .*= -1.0
    res[1:2] += z[1:2]
    return nothing
end

