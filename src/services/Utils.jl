

function intersectPlaneLine(
    planenorm::AbstractVector{<:Real}, 
    planepnt::AbstractVector{<:Real}, 
    raydir::AbstractVector{<:Real}, 
    raypnt::AbstractVector{<:Real}
)
    ndotu = dot(planenorm, raydir)
    if ndotu ≈ 0 error("no intersection or line is within plane") end

    w  = raypnt - planepnt
    si = -dot(planenorm, w) / ndotu
    ψ  = w .+ si .* raydir .+ planepnt
    return ψ
end


"""
    CameraModel(width,height,fc,cc,skew,kc)

Constructor helper for creating a camera model.
"""
function CameraSkewDistortion(width,height,fc,cc,skew,kc)
    KK = [fc[1]      skew  cc[1];
             0       fc[2] cc[2];
             0		    0     1]
    # KK = [fc[1] skew*fc[1] cc[1];
    #          0       fc[2] cc[2];
    #          0		    0     1]
    Ki = inv(KK)
    CameraModelandParameters(width,height,fc,cc,skew,kc,KK,Ki)
end