
"""
    $TYPEDEF

Standard pinhole camera model with distortion parameters (aka camera intrinsics).  

Notes:
- Image origin assumed as top-left.
- Keeping with Images.jl,
  - width of the image are matrix columns from left to right.
  - height of the image are matrix rows from top to bottom.
  - E.g. `mat[i,j] == img[h,w] == mat[h,w] == img[i,j]`
    - This is to leverage the unified Julia Arrays infrastructure, incl vectors, view, Static, CPU, GPU, etc.

Legacy Comments:
----------------

Pinhole Camera model is the most simplistic.

Notes
- https://en.wikipedia.org/wiki/Pinhole_camera
- Standard Julia *[Images.jl](https://juliaimages.org/latest/)-frame* convention is, `size(img) <==> (i,j) <==> (height,width) <==> (y,x)`,
  - Common *camera-frame* in computer vision and robotics, `(x,y) <==> (width,height) <==> (j,i)`,
  - Using top left corner of image as `(0,0)` in all cases. 
  - Direct comparison with [OpenCV convention](https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html) is:
    - `(x,y) [CamXYZ] <==> (j,i) [Images.jl] <==> (u,v) [OpenCV]` -- look very carfully at `(u,v) <==> (j,i)` in *image-frame*
- Always follow right hand rule for everything.
- A convenience Union type [`CameraCalibrationT`](@ref) is provided to develop implementations against `struct` and `mutable struct` types.

DevNotes
- https://en.wikipedia.org/wiki/Distortion_(optics)


Also see: [`CameraCalibrationT`](@ref) [`CameraCalibrationMutable`](@ref), (TODO: `ProjectiveCameraModel`)
"""
struct CameraCalibration{R <: Real,N} <: AbstractCameraModel
  """ numver of pixels from top to bottom """
  height::Int		# = 480
  """ number of pixels from left to right """
  width::Int		# = 640
  """ distortion coefficients up to fifth order """
  kc::SVector{N,R} # = zeros(5)
  """ 3x3 camera calibration matrix """
  K::SMatrix{3,3,R}   # = SMatrix{3,3}([[height;0.0;width/2]';[0.0;height;height/2]';[0.0;0;1]'] )
  """ inverse of a 3x3 camera calibration matrix """
  Ki::SMatrix{3,3,R}  # = inv(K)
end


"""
    $TYPEDEF

See [`CameraCalibraton`](@ref).
"""
mutable struct CameraCalibrationMutable{R <: Real,N} <: AbstractCameraModel
  """ numver of pixels from top to bottom """
  height::Int		# = 480
  """ number of pixels from left to right """
  width::Int		# = 640
  """ distortion coefficients up to fifth order """
  kc::MVector{R,N} # = zeros(5)
  """ 3x3 camera calibration matrix """
  K::MMatrix{3,3,R}   # = SMatrix{3,3}([[height;0.0;width/2]';[0.0;height;height/2]';[0.0;0;1]'] )
  """ inverse of a 3x3 camera calibration matrix """
  Ki::MMatrix{3,3,R}  # = inv(K)
end

"""
    CameraCalibrationT

A Union type for users to implement against both `struct`` and `mutable struct` definitions of `CameraCalibration`
"""
CameraCalibrationT = Union{<:CameraCalibration, <:CameraCalibrationMutable}

#