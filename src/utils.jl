

"""Updates `all_points` with `new_batch`, empties the `new_batch`, and rebuilds the `kdtree`."""
function update_kdtree(rrtstar::RRTStarPlanner)
    rrtstar.all_points[:, rrtstar.num_total_points+1:rrtstar.num_total_points+rrtstar.new_batch_size] .= rrtstar.new_batch
    rrtstar.num_total_points += rrtstar.new_batch_size
    rrtstar.new_batch_size = 0
    rrtstar.kdtree = KDTree(rrtstar.all_points[:, 1:rrtstar.num_total_points])
    return nothing
end

"""Queries and returns the set of tree-points that lie within `r` radius of the given point `p`. `all_points` are queried using the `kdtree` and `new_batch` points are queried by brute force. The results are combined and returned."""
function points_within_r_ball(rrtstar::RRTStarPlanner, p::Point, r::Float64)::Set{Point}
    points_within_r::Set{Point} = Set{Point}()
    if rrtstar.num_total_points > 0
        idxs = inrange(rrtstar.kdtree, collect(p), r)
        for idx::Int in idxs
            p2 = rrtstar.all_points[1, idx], rrtstar.all_points[2, idx]
            push!(points_within_r, p2)
        end
    end
    if rrtstar.new_batch_size > 0
        for i in 1:rrtstar.new_batch_size
            p2 = rrtstar.new_batch[1, i], rrtstar.new_batch[2, i]
            l2dist(p, p2) < r  &&  push!(points_within_r, p2)
        end
    end
    return points_within_r
end

"""Queries and returns the tree-point that is closest to the given point `p`. `all_points` are queried using the `kdtree` and `new_batch` points are queried by brute force. The overall closest point `p` is returned"""
function get_closest_point(rrtstar::RRTStarPlanner, p::Point)::Point
    lowest_dist::Float64 = Inf
    closest_point::Point = (Inf, Inf)
    dist::Float64 = Inf
    if rrtstar.num_total_points > 0
        idx::Int, dist = nn(rrtstar.kdtree, collect(p))
        closest_point = rrtstar.all_points[1, idx], rrtstar.all_points[2, idx]
        lowest_dist = dist
    end
    if rrtstar.new_batch_size > 0
        for i in 1:rrtstar.new_batch_size
            p2 = rrtstar.new_batch[1, i], rrtstar.new_batch[2, i]
            dist = l2dist(p, p2)
            if dist < lowest_dist
                closest_point = p2
                lowest_dist = dist
            end
        end
    end
    
    if rrtstar.new_batch_size == rrtstar.batch_size
        update_kdtree(rrtstar)
    end

    return closest_point
end


