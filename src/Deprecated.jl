

function Base.getproperty(x::Pinhole, f::Symbol)
  if f == :columns
    getfield(x, :width)
  elseif f == :rows
    getfield(x, :height)
  elseif f == :prinicipalpoint
    view(getfield(x, :K), 1:2, 3)
  elseif f == :focallength
    K = getfield(x, :K)
    SA[K[1,1];K[2,2]] 
  else
    getfield(x, f)
  end
end



# ## From yakir12/CameraModels.jl
# struct Pinhole <: AbstractCameraModel
#   # note order of elements has been changed from original source so that inner constructor can be removed.
#   columns::Int
#   rows::Int
#   prinicipalpoint::PixelCoordinate # in pixels
#   focallength::Vector2             # in pixels
# end


# abstract type CameraModel end