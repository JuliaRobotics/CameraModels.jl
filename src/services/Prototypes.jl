
"""
    origin(ray)

Return the direction of the ray as a (Vector3)
"""
function origin end

origin(vector::Vector3) = origin3d
origin(ray::Ray) = vector.origin


"""
    direction(ray)

Return the origin of ray, typically just a zero $(Point3) for normal cameras.
"""
function direction end

direction(vector::Vector3) = vector
direction(ray::Ray) = vector.direction


"""
    columns(model::CameraModel)

Returns the width of the camera sensor.
"""
function columns end

"""
    rows(model::CameraModel)

Returns the height of the camera sensor.
"""
function rows end

"""
    sensorsize(model::CameraModel)

Return the size of the camera sensor. By default calling out to columns(model) and rows(model) to build an SVector{2}

`sensorsize(cameramodel::CameraModel) = SVector{2}(columns(cameramodel), rows(cameramodel))`
"""
function sensorsize end

sensorsize(cameramodel::CameraModel) = SVector{2}(columns(cameramodel), rows(cameramodel))

"""
    pixel2ray(cameramodel::CameraModel, pixelcoordinate::$(PixelCoordinate))

Returns the ray in space (origin + direction) corresponding to this `pixelcoordinate`.
"""
function pixel2ray end

"""
    point2pixel(camera::CameraModel, pointincamera::$(Point3))

Returns the pixel location onto which the 3D coordinate `pointincamera` is projected.
"""
function point2pixel end

"""
    lookdirection(camera::CameraModel)

Return the lookdirection of this camera model.
"""
function lookdirection end

"""
    updirection(camera::CameraModel)

Return the updirection of this camera model.
"""
function updirection end

"""
    canreproject(camera::CameraModel)

Confirms if point2pixel is implemented for this camera model.
"""
canreproject(camera::CameraModel) = true