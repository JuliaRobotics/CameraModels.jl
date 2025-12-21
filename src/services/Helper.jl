"""
    $(SIGNATURES)

Constructor helper assuming you just have a camera image and need to start somewhere for a basic camera model.

Notes:
- Calibration will incorrect but hopefully in a distant ballpark to get the calibration process started.
- See [AprilTags.jl Calibration section](https://juliarobotics.org/AprilTags.jl/latest/#Camera-Calibration-1) for code and help.
"""
function CameraCalibration(img::AbstractMatrix{T}) where {T}
    height, width = size(img)
    # emperical assumption usually seen for focal length
    f_w = f_h = 1.1 * height
    c_w, c_h = width / 2, height / 2
    K = @SMatrix [
        f_w   0.0   c_w
        0.0   f_h   c_h
        0.0   0.0   1.0
    ]
    @info "Assuming default CameraCalibration from image size(img)=(rows,cols)=$(size(img)):" f_w f_h c_w c_h
    return CameraCalibration(; width, height, K)
end

"""
    $(SIGNATURES)

Mutable version of CameraCalibration(img) helper function.
"""
function CameraCalibrationMutable(img::AbstractMatrix{T}) where {T}
    height, width = size(img)
    # emperical assumption usually seen for focal length
    f_w = f_h = 1.1 * height
    c_w, c_h = width / 2, height / 2
    K = @MMatrix [
        f_w   0.0   c_w
        0.0   f_h   c_h
        0.0   0.0   1.0
    ]
    @info "Assuming default CameraCalibrationMutable from image size(img)=(rows,cols)=$(size(img)):" f_w f_h c_w c_h
    return CameraCalibrationMutable(; width, height, K)
end
