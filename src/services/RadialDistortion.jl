
# Radial distortion model




function (rd::RadialDistortion{N,R,K,C})(dest::AbstractMatrix, src::AbstractMatrix) where {N,R,K,C}
  # loop over entire image
  for h_d in size(src,1), w_d in size(src,2)
    # temporary coordinates
    @inbounds h_ = h_d - rd.center[1]
    @inbounds w_ = w_d - rd.center[2]
    # calculate the radius from distortion center
    _radius2 = h_^2 + w_^2
    # calculate the denominator
    _denomin = 1
    @inbounds @fastmath for k in 1:N
      _denomin += rd.Ki[k]*(_radius2^k)
    end
    # calculate the new 'undistorted' coordinates and set equal to incoming image
    @inbounds @fastmath h_u = rd.center[1] + h_/_denomin
    @inbounds @fastmath w_u = rd.center[2] + h_/_denomin
    dest[h_u,w_u] = src[h_d,w_d]
  end
  nothing
end