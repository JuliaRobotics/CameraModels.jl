
"""
    origin(ray)

Return the direction of the ray as a (Vector3)
"""
function origin end

"""
    direction(ray)

Return the origin of ray, typically just a zero $(Point3) for normal cameras.
"""
function direction end

"""
    columns(model::AbstractCameraModel)

Returns the width of the camera sensor.
"""
function width end

"""
    rows(model::AbstractCameraModel)

Returns the height of the camera sensor.
"""
function height end

"""
    sensorsize(model::AbstractCameraModel)

Return the size of the camera sensor. By default calling out to columns(model) and rows(model) to build an SVector{2}

`sensorsize(cameramodel::AbstractCameraModel) = SVector{2}(columns(cameramodel), rows(cameramodel))`
"""
function sensorsize end

"""
    pixel2ray(cameramodel::AbstractCameraModel, pixelcoordinate::$(PixelCoordinate))

Returns the ray in space (origin + direction) corresponding to this `pixelcoordinate`.
"""
function pixel2ray end

"""
    point2pixel(camera::AbstractCameraModel, pointincamera::$(Point3))

Returns the pixel location onto which the 3D coordinate `pointincamera` is projected.
"""
function point2pixel end

"""
    lookdirection(camera::AbstractCameraModel)

Return the lookdirection of this camera model.
"""
function lookdirection end

"""
    updirection(camera::AbstractCameraModel)

Return the updirection of this camera model.
"""
function updirection end

