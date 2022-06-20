
const Point = Tuple{Float64,Float64}  # x, y

abstract type AbstractObstacle end

struct Line <: AbstractObstacle
    p1::Point
    p2::Point
end

struct Circle <: AbstractObstacle
    center::Point
    radius::Float64
end

struct Rect <: AbstractObstacle
    topleft::Point
    size::Tuple{Float64,Float64}  # width , height
end

Rect(topleft::Point, side::Float64) = Rect(topleft, (side, side))


"""Gets corners points of the given rectangle as the tuple (topleft, topright, bottomleft, bottomright)"""
function corners(rect::Rect)::NTuple{4,Point}
    return (rect.topleft,  # topleft
            rect.topleft .+ (rect.size[1], 0.0),  # top right
            rect.topleft .+ (0, rect.size[2]),  # bottom left
            rect.topleft .+ rect.size  # bottom right
            )
end

"""Gets edges of the given rectangle as a tuple of line segments (topedge, leftedge, bottomedge, rightedge)"""
function edges(rect::Rect)::NTuple{4,Line}
    topleft, topright, bottomleft, bottomright = corners(rect)
    return (
        Line(topleft, topright),  # top edge
        Line(topleft, bottomleft),  # left edge
        Line(bottomleft, bottomright),  # bottom edge
        Line(topright, bottomright)  # right edge
    )
end



"""L₂ norm of the given point"""
l2norm(p::Point) =  √(sum(p.^2))

"""Euclidean distance between two points"""
l2dist(p1::Point, p2::Point) = l2norm(p1 .- p2)



"""
Checks whether the two lines intersect.
"""
function intersects(l1::Line, l2::Line)::Bool
    (x₁, y₁), (x₂, y₂) = l1.p1, l1.p2
    (x₃, y₃), (x₄, y₄) = l2.p1, l2.p2
    
    a = (y₃ - y₄) * (x₁ - x₃) + (x₄ - x₃) * (y₁ - y₃)
    b = (y₁ - y₂) * (x₁ - x₃) + (x₂ - x₁) * (y₁ - y₃)
    denom = (x₄ - x₃) * (y₁ - y₂) - (x₁ - x₂) * (y₄ - y₃)
    
    abs(denom) < 0.0001 && return true  # colinear

    return  0 <= a / denom <= 1  &&  0 <= b / denom <= 1
end


"""
Checks whether the given line segment intersects with the given circle.\\
Line segment `((x₁,y₁),(x₂,y₂))` and circle `((x₀,y₀),r)` intersect if for some 0 ≤ t ≤ 1: \\
`at² + bt + c ≤ 0` \\
where: \\
`a = (x₁-x₂)² + (y₁-y₂)²` \\
`b = 2[(x₁-x₂)(x₂-x₀) + (y₁-y₂)(y₂-y₀)]` \\
`c = (x₂-x₀)² + (y₂-y₀)² - r²`
"""
function intersects(line::Line, circle::Circle)::Bool
    (x₁, y₁), (x₂, y₂) = line.p1, line.p2
    (x₀, y₀), r = circle.center, circle.radius

    if l2dist((x₁, y₁), (x₀, y₀)) <= r  || l2dist((x₂, y₂), (x₀, y₀)) <= r
        return true  # one or both points are on/in the circle
    end

    a = (x₁ - x₂)^2 + (y₁ - y₂)^2
    b = 2 * ((x₁ - x₂) * (x₂ - x₀) + (y₁ - y₂) * (y₂ - y₀))
    c = (x₂ - x₀)^2 + (y₂ - y₀)^2 - r^2

    Δ = b^2 - 4 * a * c  # discriminant
    if Δ < 0
        return false
    else
        roots = (-b + √Δ) / (2 * a),  (-b - √Δ) / (2 * a)  # intersection points
        return any(0 .<= roots .<= 1)
    end
end


"""Checks whether the given line segment intersects with the given rectangle. The logic is to test intersection with each edge of the rectangle"""
function intersects(line::Line, rect::Rect)::Bool
    for e in edges(rect)
        if intersects(line, e)
            return true
        end
    end
    return false
end
