



function count_saturated_N0f8(img)
  sum((s->s == N0f8(1)).(img))
end

function percentile_saturated_RGB(img)
  perc = 0.0
  perc += count_saturated_N0f8((s->s.r).(img))
  perc += count_saturated_N0f8((s->s.g).(img))
  perc += count_saturated_N0f8((s->s.b).(img))
  perc /= 3
  perc /= *(size(img)...)
  return perc
end



function estimateShutterTimes_RGB(
  imgs::AbstractVector{<:AbstractMatrix},
  nominal::Real = 1/100,
)
  avgEk = []
  scalek = []
  for img in imgs
    avgE_ = (s->(s.r + s.g + s.b)/3)(mean(img))
    push!(avgEk, avgE_)
    # many saturated pixels mean stronger scaling on nominal / avgE
    push!(scalek, exp(percentile_saturated_RGB(img)) )
  end
  # avg energy in scene
  avgE = mean(avgEk)

  ((nominal / avgE ) .* avgEk) .* scalek
end

function percentile_saturated_gray(img)
  perc = 0.0
  perc += count_saturated_N0f8((s->s.val).(img))
  perc /= *(size(img)...)
  return perc
end

function estimateShutterTimes_gray(
  imgs::AbstractVector{<:AbstractMatrix},
  nominal::Real = 1/100,
)
  avgEk = []
  scalek = []
  for img in imgs
    avgE_ = (s->s.val)(mean(img))
    push!(avgEk, avgE_)
    # many saturated pixels mean stronger scaling on nominal / avgE
    push!(scalek, exp(percentile_saturated_gray(img)) )
  end
  # avg energy in scene
  avgE = mean(avgEk)

  ((nominal / avgE ) .* avgEk) .* scalek
end


"""
_pixvalWindow

A utility weighting function for HDR smoothness regularization.

See: solveDetectorResponse for more details.
"""
function _pixvalWindow(
  z::Int;
  zmin::Int = 0,
  zmax::Int = 2^8 - 1,
)
  if z <= (zmin + zmax)/2
    return z - zmin
  else
    return zmax - z
  end
end


"""
solveDetectorResponse

Solve for imaging system response function, g[z] which is the log exposure
corresponding to pixel value z:
  g := ln(f^-1)  :  pixelval --> log irradiance + log exposure.

This procedure uses various pixel values Z[j][i] from a set of physical scene 
objects i, as repeatedly observed with different accumulated energies j.  
logΔTs[j] is a vector of the log delta t / shutter speeds / ambient radiance.  
Note, log exposure[i] is the log film irradiance at pixel location i

The core assumption is that the different pixel values from 
different images can be attributed to the same underlying physical 
scene object with known received energy (exposure time).  This allows for an 
optimization to be performed to solve for the imaging system 
response function gcurve. Various physical scene locations i intend to 
make the whole pixel value range of gcurve to be sufficiently observed.

Unscaled radiance is reconstructed using (gcurve[zij] - logΔts[j]) or better,
  - see `recoverRadianceWeighted`, `_pixvalWindow`

How different pixels of the same physical scene object are observed is
less important, barring image warping/perspective limitations.  This procedure
is not capable of solving both object albedo and environmental changes simultaneously.

Returns a tuple (gcurve, logExposure)

Parameters:
- λ::Real is the regularization constant rewarding camera response curve smoothness.
- window: z --> weight, relative importance of different mid-range pixel values for the optimization.
  - see _pixvalWindow(z) for a common weighting function that de-emphasizes near saturation pixel values.


Usage example:
```julia
# stack images of the same scene with different exposures
imgs = [img1, img2, img3]
# extract average exposure as proxy for shutter speeds
est_lΔT = log.(CameraModels.estimateShutterTimes_RGB(imgs))
# Z[j][i] is the pixel values of pixel location number i in image j
_getpixel(mg, ij) = mg[ij...]
_collectpixs(mg) = _getpixel.(Ref(mg), uv)
_Z = _collectpixs.(imgs)

# Convert FixedPoint N0f8 values to UInt8 (todo all channels)
Zr = (s->((p->reinterpret(UInt8,p.r)).(s))).(_Z)
gr, lEr = CameraModels.solveDetectorResponse(Zr, est_lΔT; λ)
# ...
normalized_image = CameraModels.recoverRadianceWeighted_RGB(imgs, est_lΔT, [gr, gg, gb])
```

See also: `randomPixels`, `normalizeRadianceMap`, `recoverRadianceWeighted_RGB`, `_pixvalWindow`
"""
function solveDetectorResponse(
  Z::AbstractVector,
  logΔTs::AbstractVector;
  n::Int = 2^8, # common for 3*8 bit RGB;
  window::AbstractVector{<:Real} = _pixvalWindow.(0:(n-1)),
  λ::Real = 200.0,
  diff_kernel::AbstractVector{<:Real} = SA[1, -2, 1],
)
  # System size
  nimgs = length(Z)
  nlocs = length(Z[1])
  
  # Prepare a linear system.
  # The total number of equations (neqs), which includes:
  # - nlocs*nimgs equations from pixel value observations
  # - 1 equation from fixing gcurve middle value to 0 for scale
  # - n equations from smoothness regularization
  # A is size (neqs, Zmax-Zmin+pixel_locations)
  # A = [A1 A2; A3 0], where
  # A{1,2} is has rows that each contain one column entries, gcurve[zij] = logExposure_ij + logΔt_j
  # - a1: the weight at column z (observed pixel value) and next available column 
  # - a2: the negative weight at column n+i (corresponding to list_i of pixel locations)
  #   A2 is also off-diagonal
  # A3 is the weighted regularization as off diagnonal entries for smoothness of gcurve, and 
  # b is size (neqs) = [weighted logΔTs; zeros(n)]
  # x = [gcurve; logExposure] = A * b
  neqs = nlocs*nimgs + 1 + n
  A = zeros(neqs,n+nlocs)
  b = zeros(neqs)

  # Fill in the equations from pixel value observations in A and b,
  # Note the number of equations is nlocs*nimgs, where each pixel value observation contributes one equation.
  k = 1;
  for i in 1:nlocs, j in 1:nimgs
    # A1 columns correspond to gcurve possible pixel z values, 
    # A2 columns correspond to logExposure of observed pixels from list of locations i
    pixz1 = Z[j][i] + 1
    a_12 = view(A, k, SA[pixz1, n+i])
    a_12 .= window[pixz1] .* SA[1, -1]
    b[k] = window[pixz1] * logΔTs[j]
    k += 1
  end

  # Add the equation to fix the curve by setting its middle value to 0
  # Assumption, middle value between (Zmax-min)/2 + 1, e.g. [0..255]
  A[k,Int(n//2 + 1)] = 1;
  k += 1

  # Add n equations for smoothness regularization, which encourages the 
  # response curve to be smooth by penalizing second derivatives.
  for i in 2:n−1
    # A3 off-diagonal entries numerically approximate the second derivative of gcurve at possible pixel values 
    _A = view(A, k, (i-1):(i+1))
    _A .= λ*window[i] .* diff_kernel
    k += 1
  end

  # Solve linear system via Cholesky, QR, similar (internal Julia solver)
  x = A\b

  # Extract gcurve and log exposure values from solution vector
  gcurve = x[1:n]
  logExposure = x[(n+1):end]
  return gcurve, logExposure
end


function normalizeRadianceMap(
  rm, 
  nper::Real = 0.02;
  upper = maximum(rm),
)
  lower = percentile(rm[:], nper)
  nrm = (rm .- lower) ./ (upper - lower)
  nrm[nrm .< 0] .= 0
  return nrm, upper
end


function recoverRadianceWeighted(
  imgs,
  logΔts,
  gcurve;
  window::AbstractVector{<:Real} = _pixvalWindow.(0:(2^8 - 1)),
  normalize_percentile::Real = 0.02, # set <= 0 to disable
)
  resim = zeros(size(imgs[1])...)
  wmap = zeros(size(imgs[1])...)

  for (k,img) in enumerate(imgs)
    for i in axes(img,1)
      for j in axes(img,2)
        pixel = img[i,j]
        zij = Int16(reinterpret(UInt8,pixel))+1
        wij = window[zij]
        resim[i,j] += wij*(gcurve[zij] - logΔts[k])
        wmap[i,j] += wij
      end
    end
  end

  rim = resim ./ (wmap .+ 1)
  if 0 < normalize_percentile
    return normalizeRadianceMap(rim, normalize_percentile)[1]
  else
    return rim
  end
end


function recoverRadianceWeighted_RGB(
  imgs,
  logΔts,
  gcurve_rgb;
  window::AbstractVector{<:Real} = _pixvalWindow.(0:(2^8 - 1)),
  normalize_percentile::Real = 0.02, # set <= 0 to disable
)
  imgs_r = (_img->((s->s.r).(_img))).(imgs)
  imgs_g = (_img->((s->s.g).(_img))).(imgs)
  imgs_b = (_img->((s->s.b).(_img))).(imgs)

  rim_r = recoverRadianceWeighted(imgs_r, logΔts, gcurve_rgb[1]; window, normalize_percentile) # = 0);
  rim_g = recoverRadianceWeighted(imgs_g, logΔts, gcurve_rgb[2]; window, normalize_percentile) # = 0);
  rim_b = recoverRadianceWeighted(imgs_b, logΔts, gcurve_rgb[3]; window, normalize_percentile) # = 0);

  # upper = maximum(vcat(rim_r[:], rim_g[:], rim_b[:]))
  # rim_r, = normalizeRadianceMap(rim_r, 0.02; upper)
  # rim_g, = normalizeRadianceMap(rim_g, 0.02; upper)
  # rim_b, = normalizeRadianceMap(rim_b, 0.02; upper)

  return colorview(RGB, rim_r, rim_g, rim_b)
end

"""
randomPixels

Utility function to sample N random pixel locations from an image, which can be used for HDR curve solving.

See also: `solveDetectorResponse`, `recoverRadianceWeighted`
"""
function randomPixels(
  img::AbstractMatrix,
  N::Int = 1
)
  uu = rand(1:size(img,1), N)
  vv = rand(1:size(img,2), N)
  zip(uu,vv) |> collect
end