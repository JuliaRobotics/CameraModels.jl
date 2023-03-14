
# Example: Ray trace

```julia
## load packages
using CameraModels
using Manifolds

## Camera setup
f = 800.0        # pixels
ci,cj = 360,640  # assuming 720x1280 image
# going from imaging array to camera frame
c_H_a = [0 1 -cj; 1 0 -ci; 0 0 f] # camera matrix


## Setup for coordinate operations 
M = SpecialEuclidean(3)
Mr = M.manifold[2]
R0 = [1 0 0; 0 1 0; 0 0 1.]
e0 = ArrayPartition([0;0;0.], R0)

# Place the body somewhere in the world
w_T_b = ArrayPartition([0.;0.;2.], exp_lie(Mr, hat(Mr, R0, [0;0;0.])))
# place the camera somewhere in the body frame
b_T_ex = ArrayPartition([0.;0.;0.], exp_lie(Mr, hat(Mr, R0, [0;0.2;0.2])))
ex_T_c = ArrayPartition([0.;0.;0.], [0 0 1; -1 0 0; 0 -1 0.])
b_T_c = compose(M, b_T_ex, ex_T_c)
# w_T_c = compose(M, w_T_b, b_T_c)


## convert pixel location to ray in camera then body frame
a_Forb = [360; 640; 1.0]
c_Forb = c_H_a * a_Forb
# unit vector ray direction from camera in camera coordinates 
c_uV = c_Forb ./ norm(c_Forb)
# get ray direction in body coordinates
b_uV = b_T_c.x[2] * c_uV

# and ray position from camera center in body frame
b_P = b_T_c.x[1]


## planar floor surface projection
b_nFL = [0; -0.05; 1.] # skipping floor pitch or roll information
b_nFL ./= norm(b_nFL)
b_FL = [0; 0; -2.]

b_Forb = intersectLineToPlane3D(b_nFL, b_FL, b_uV, b_P)

## find feature points in the world frame
_w_Forb = affine_matrix(M, w_T_b)*[b_Forb; 1.]
```