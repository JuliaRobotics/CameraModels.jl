

"""
Data structure for a Camera model with parameters.
Use `CameraModel(width,height,fc,cc,skew,kc)` for easy construction.
"""
struct CameraModelandParameters
    width::Int		# image width
    height::Int		# image height
    fc::Vector{Float64}	# focal length in x and y
    cc::Vector{Float64}	# camera center
    skew::Float64	    # skew value
    kc::Vector{Float64} # distortion coefficients up to fifth order
    K::Matrix{Float64} # 3x3 camera calibration matrix (Camera intrinsics)
    Ki::Matrix{Float64} # inverse of a 3x3 camera calibratio matrix
end