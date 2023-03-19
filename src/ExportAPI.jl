


export AbstractCameraModel

export CameraCalibration, CameraCalibrationMutable

export CameraSkewDistortion

export Ray, PixelIndex
export pixel2ray, point2pixel, canreproject, sensorsize #, origin, direction, 
export project, projectHomogeneous
export backproject, backprojectHomogeneous

export lookdirection, updirection #, columns, rows

export intersectLineToPlane3D, intersectRayToPlane

# suppressing super general signatures likely to have conflicts.
# TODO adopt common Julia definition for points and vectors, maybe something from JuliaGeometry, etc.
# export Vector3, Vector2, Point3