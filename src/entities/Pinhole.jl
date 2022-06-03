
## FIXME consolidation necessary

## From yakir12/CameraModels.jl
struct Pinhole <: CameraModel
    
  Pinhole(pixelwidth::Int, pixelheight::Int, prinicipalpoint::PixelCoordinate, focallength::Vector2) = new(prinicipalpoint, focallength, pixelwidth, pixelheight)

  prinicipalpoint::PixelCoordinate # in pixels
  focallength::Vector2 # in pixels
  columns::Int
  rows::Int
end


## From JuliaRobotics/Caesar.jl
"""
    $TYPEDEF

Pinhole Camera model is the most simplistic.

Notes
- https://en.wikipedia.org/wiki/Pinhole_camera
- Standard Julia *[Images.jl](https://juliaimages.org/latest/)-frame* convention is, `size(img) <==> (i,j) <==> (height,width) <==> (y,x)`,
  - Common *camera-frame* in computer vision and robotics, `(x,y) <==> (width,height) <==> (j,i)`,
  - Using top left corner of image as `(0,0)` in all cases. 
  - Direct comparison with [OpenCV convention](https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html) is:
    - `(x,y) [CamXYZ] <==> (j,i) [Images.jl] <==> (u,v) [OpenCV]` -- look very carfully at `(u,v) <==> (j,i)` in *image-frame*
- Always follow right hand rule for everything.

DevNotes
- https://en.wikipedia.org/wiki/Distortion_(optics)
"""
struct PinholeCamera{R <: Real} <: AbstractCameraModel
  K::SMatrix{3,3,R}
end