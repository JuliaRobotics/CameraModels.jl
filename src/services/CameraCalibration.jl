
function Base.show(io::IO, ::MIME"text/plain", cam::CameraCalibration)
  println(io,"CameraCalibration {")
  println(io,"  sensorsize      (w,h) = ", sensorsize(cam))
  println(io,"  principal_point (w,h) = ", pp_w(cam), ",", pp_h(cam))
  println(io,"  focal_length    (w,h) = ", f_w(cam), ",", f_h(cam))
  println(io,"  shear                 = ", shear(cam))
  println(io,"  radtan coeff          = ", cam.kc)
  println(io, "}")
  nothing
end

Base.show(io::IO, ::MIME"text/markdown", cam::CameraCalibration) = show(io, MIME("text/plain"), cam)
Base.show(io::IO, cam::CameraCalibration) = show(io, MIME("text/plain"), cam)



"""
    $SIGNATURES

Constructor helper assuming you just have a camera image and need to start somewhere for a basic camera model.

Notes:
- Calibration will incorrect but hopefully in a distant ballpark to get the calibration process started.
- See [AprilTags.jl Calibration section](https://juliarobotics.org/AprilTags.jl/latest/#Camera-Calibration-1) for code and help. 
"""
function CameraCalibrationMutable(img::AbstractMatrix{T}) where T
  height, width = size(img)
  # emperical assumption usually seen for focal length
  f_w = 1.1*height
  f_h = f_w
  c_w, c_h = width/2, height/2
  K = MMatrix{3,3}([[f_w;0.0;c_w]';[0.0;f_h;c_h]';[0.0;0;1.]'] )
  @info "Assuming default CameraCalibrationMutable from image size(img)=(rows,cols)=$(size(img)):" f_w f_h c_w c_h
  CameraCalibrationMutable(;width, height, K)
end

