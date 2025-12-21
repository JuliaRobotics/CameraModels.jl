function Base.show(io::IO, ::MIME"text/plain", cam::CameraCalibration)
    println(io, "CameraCalibration {")
    println(io, "  sensor size     (width, height) = ", sensorsize(cam))
    println(io, "  principal point (width, height) = ", pp_w(cam), ", ", pp_h(cam))
    println(io, "  focal length    (width, height) = ", f_w(cam), ", ", f_h(cam))
    println(io, "  shear coefficient               = ", shear(cam))
    println(io, "  radial/tangential coefficients  = ", cam.kc)
    println(io, "}")
    return nothing
end

Base.show(
    io::IO,
    ::MIME"text/markdown",
    cam::CameraCalibration
) = show(io, MIME("text/plain"), cam)

Base.show(
    io::IO,
    cam::CameraCalibration
) = show(io, MIME("text/plain"), cam)


function Base.show(io::IO, ::MIME"text/plain", cam::CameraCalibrationMutable)
    println(io, "CameraCalibrationMutable {")
    println(io, "  sensor size     (width, height) = ", sensorsize(cam))
    println(io, "  principal point (width, height) = ", pp_w(cam), ", ", pp_h(cam))
    println(io, "  focal length    (width, height) = ", f_w(cam), ", ", f_h(cam))
    println(io, "  shear coefficient               = ", shear(cam))
    println(io, "  radial/tangential coefficients  = ", cam.kc)
    println(io, "}")
    return nothing
end

Base.show(
    io::IO,
    ::MIME"text/markdown",
    cam::CameraCalibrationMutable
) = show(io, MIME("text/plain"), cam)

Base.show(
    io::IO,
    cam::CameraCalibrationMutable
) = show(io, MIME("text/plain"), cam)
