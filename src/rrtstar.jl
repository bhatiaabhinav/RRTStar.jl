using Distributions
using LinearAlgebra
using NearestNeighbors
using Random

"""Rapidly Exploring Random Tree (RRT*) Search Algorithm"""
mutable struct RRTStarPlanner
    # The parameters that define an instance of RRT*:
    x_range::Tuple{Float64,Float64}
    y_range::Tuple{Float64,Float64}
    start::Point
    goal::Point
    obstacles::Set{AbstractObstacle}
    max_samples::Int
    growth_factor::Float64
    focus_level::Float64
    focus_index::Tuple{Int,Int}
    goal_bias::Float64
    batch_size::Int
    rng::MersenneTwister  # for sampling
    
    # Internal Parameters:
    parents::Dict{Point,Union{Nothing,Point}}
    children::Dict{Point,Set{Point}}
    costs::Dict{Point,Float64}
    num_points_sampled::Int
    all_samples::Matrix{Float64}
    num_total_points::Int  # accepted points
    all_points::Matrix{Float64}  #  Matrix{Float64} = Array{Float64, 2} i.e., 2D array. Column vectors.
    num_improving_points::Int 
    improving_points::Matrix{Float64} 
    new_batch::Matrix{Float64}  #  Matrix{Float64} = Array{Float64, 2} i.e., 2D array. Column vectors.
    new_batch_size::Int
    kdtree::KDTree

    dist_to_goal_lb::Float64  # of the point closest to the goal
    focus_area_scores::Matrix{Float32}
    viz_r::Float64
    viz_new_point::Union{Point,Nothing}

    function RRTStarPlanner(x_range::Tuple{Float64,Float64}, y_range::Tuple{Float64,Float64}, start::Point, goal::Point, obstacles::Set{AbstractObstacle}, max_samples::Int; growth_factor::Float64 = 1.0, goal_bias::Float64 = 0.01, batch_size::Int = 100, focus_level::Float64 = 0.0, focus_index::Tuple{Int,Int} = (1,4))
        rrt = new(x_range, y_range, start, goal, obstacles, max_samples, growth_factor, focus_level, focus_index, goal_bias, batch_size, MersenneTwister())
        rrt.parents = Dict{Point,Union{Nothing,Point}}()
        rrt.children = Dict{Point,Set{Point}}()
        rrt.costs = Dict{Point,Float64}()
        rrt.costs[start] = 0
        rrt.costs[goal] = Inf
        rrt.num_points_sampled = 0
        rrt.all_samples = Matrix{Float64}(undef, 2, max_samples)
        rrt.all_points = Matrix{Float64}(undef, 2, max_samples)
        rrt.num_total_points = 0
        rrt.new_batch = Matrix{Float64}(undef, 2, rrt.batch_size)
        rrt.new_batch_size = 0
        rrt.kdtree = KDTree(rrt.all_points[:, 1:1])  # default leafsize is 10 and uses euclidean distance metric
        rrt.dist_to_goal_lb = l2dist(start, goal)
        rrt.improving_points = Matrix{Float64}(undef, 2, max_samples)
        rrt.num_improving_points = 0
        rrt.focus_area_scores = zeros(Float32, 20, 4) # Full features
        rrt.viz_r = 0
        rrt.viz_new_point = nothing
        return rrt
    end
end

cost(rrtstar::RRTStarPlanner)::Float64 = rrtstar.costs[rrtstar.goal]

"""Samples a point from the focus region"""
function focused_sample(rrt::RRTStarPlanner)::Point
    x_size = rrt.x_range[2] - rrt.x_range[1]
    y_size = rrt.y_range[2] - rrt.y_range[1]
    x_bin_size = x_size / 4.0
    y_bin_size = y_size / 4.0
    new_x_range = (rrt.x_range[1] + (rrt.focus_index[1] - 1) * x_bin_size, rrt.x_range[1] + (rrt.focus_index[1]) * x_bin_size)
    new_y_range = (rrt.y_range[1] + (rrt.focus_index[2] - 1) * y_bin_size, rrt.y_range[1] + (rrt.focus_index[2]) * y_bin_size)
    return (rand(rrt.rng, Uniform(new_x_range[1], new_x_range[2])), rand(rrt.rng, Uniform(new_y_range[1], new_y_range[2])))
end


"""Samples a point within the focus region with `rrt.focus_level` probability, else samples uniformly from the entire map. When that happens, the goal point is sampled with `rrt.goal_bias` probability."""
function sample(rrt::RRTStarPlanner)::Point
    if rand(rrt.rng) < rrt.focus_level
        return focused_sample(rrt)
    end
    if rand(rrt.rng) < rrt.goal_bias && rrt.costs[rrt.goal] == Inf
        return rrt.goal
    else
        return (rand(rrt.rng, Uniform(rrt.x_range[1], rrt.x_range[2])), rand(rrt.rng, Uniform(rrt.y_range[1], rrt.y_range[2])))
    end
end


"""Relocates the sampled point `new_point` to an appropriate place such that it lies within the `growth_factor` distance from the `closest_point` in the tree, if it was not already within that distance. The relocation is done in the direction of the closest point.""" 
function steer(rrtstar::RRTStarPlanner, closest_point, new_point)::Point
    if l2dist(closest_point, new_point) < rrtstar.growth_factor
        return new_point
    else
        steer_vec = (new_point .- closest_point) ./ l2dist(new_point, closest_point)
        new_point = closest_point .+ rrtstar.growth_factor .* steer_vec
        return new_point
    end
end


"""Returns true if the line segment defined by points p1 and p2 do not intersect with any obstacle in the map"""
function obstacle_free(rrtstar::RRTStarPlanner, p1::Point, p2::Point)::Bool
    for obstacle in rrtstar.obstacles
        if intersects(Line(p1, p2), obstacle)
            return false
        end
    end
    return true
end


"""Given the nodes `updated_nodes` and the error `init_cost_deltas` in their costs, the error is corrected for and corrections are flowed down to their children. """
function update_costs(rrtstar::RRTStarPlanner, updated_nodes::Array{Point}, init_cost_deltas::Array{Float64})
    nodes_to_update = updated_nodes
    cost_deltas = init_cost_deltas
    while length(nodes_to_update) > 0
        node = popfirst!(nodes_to_update)
        if node == rrtstar.goal
            rrtstar.num_improving_points += 1
            rrtstar.improving_points[:, rrtstar.num_improving_points] .= rrtstar.viz_new_point
        end
        cost_delta = popfirst!(cost_deltas)
        rrtstar.costs[node] = rrtstar.costs[node] - cost_delta
        for child in rrtstar.children[node]
            push!(nodes_to_update, child)
            push!(cost_deltas, cost_delta)
        end
    end
    return nothing
end


"""The core logic for adding/rejecting a sampled point `orig_new_point` to the current tree"""
function extend(rrtstar::RRTStarPlanner, orig_new_point)
    updated_nodes = Point[]
    delta_costs = Float64[]

    closest_point = get_closest_point(rrtstar, orig_new_point)
    new_point = steer(rrtstar, closest_point, orig_new_point)

    rrtstar.all_samples[:, rrtstar.num_points_sampled] .= new_point

    if obstacle_free(rrtstar, closest_point, new_point)
        rrtstar.new_batch_size += 1
        rrtstar.new_batch[:, rrtstar.new_batch_size] .= new_point
        if l2dist(new_point, rrtstar.goal) < rrtstar.dist_to_goal_lb
            rrtstar.dist_to_goal_lb = l2dist(new_point, rrtstar.goal)
        end
        
        least_cost_node = closest_point
        least_cost = rrtstar.costs[least_cost_node] + l2dist(new_point, least_cost_node)

        # To set r, see page 4 and theorem 15 of "Incremental Sampling-based Algorithms for Optimal Motion Planning"
        num_points = rrtstar.num_total_points + rrtstar.new_batch_size
        
        # I am assuming that µ(Xfree) in the paper refers to the volume of free space in the environment.
        # After having looked through the paper and the proofs this is the only interpretation that makes sense, although it is never explicitly mentioned.
        γ = 6 * ((rrtstar.x_range[2] - rrtstar.x_range[1]) * (rrtstar.y_range[2] - rrtstar.y_range[1]))
        r = min(sqrt((γ / π) * (log(num_points) / num_points)), rrtstar.growth_factor)
        rrtstar.viz_r = r

        points_within_r = points_within_r_ball(rrtstar, new_point, r)
        for node in points_within_r
            if node != new_point && obstacle_free(rrtstar, node, new_point)
                cost = rrtstar.costs[node] + l2dist(new_point, node)
                if cost < least_cost
                    least_cost = cost
                    least_cost_node = node
                end
            end
        end

        
        @assert least_cost_node != new_point "least cost node was equal to new_point"

        rrtstar.parents[new_point] = least_cost_node
        rrtstar.viz_new_point = new_point
        initial_solution = false
        if new_point == rrtstar.goal && rrtstar.costs[rrtstar.goal] == Inf
            initial_solution = true
        end
        rrtstar.costs[new_point] = least_cost
        push!(rrtstar.children[least_cost_node], new_point)
        rrtstar.children[new_point] = Set{Point}()

        for node in points_within_r
            if node != least_cost_node && node != new_point
                if obstacle_free(rrtstar, node, new_point) && rrtstar.costs[node] > rrtstar.costs[new_point] + l2dist(new_point, node)
                    # Get old parent, before overwriting
                    old_parent = rrtstar.parents[node]
                    # Set new parent
                    rrtstar.parents[node] = new_point
                    # Delete old child pointer:
                    delete!(rrtstar.children[old_parent], node)
                    # Add new child pointer
                    push!(rrtstar.children[new_point], node)
                    # Add to list to update costs
                    delta_cost = rrtstar.costs[node] - (rrtstar.costs[new_point] + l2dist(node, new_point))
                    push!(updated_nodes, node)
                    push!(delta_costs, delta_cost)
                end
            end
        end
        if initial_solution
            path = extract_best_path_reversed(rrtstar)
            for i in 2:(length(path) - 1)
                rrtstar.num_improving_points += 1
                rrtstar.improving_points[:, rrtstar.num_improving_points] .= path[i]
            end
        end
    end
    if length(delta_costs) > 0
        update_costs(rrtstar, updated_nodes, delta_costs)
    end
    return nothing
end


"""Seeds the random number genrator of the RRT* algorithm"""
function Random.seed!(rrtstar::RRTStarPlanner, seed)
    Random.seed!(rrtstar.rng, seed)
end


"""Resets the data in the rrtstar object to prepare it for a new run, potentially on a different map. The function must be invoked before staring a new run."""
function reset!(rrtstar::RRTStarPlanner)
    empty!(rrtstar.parents)
    rrtstar.parents[rrtstar.start] = nothing

    empty!(rrtstar.children)
    rrtstar.children[rrtstar.start] = Set{Point}()

    empty!(rrtstar.costs)
    rrtstar.costs[rrtstar.start] = 0
    rrtstar.costs[rrtstar.goal] = Inf

    fill!(rrtstar.all_samples, 0)
    fill!(rrtstar.improving_points, 0)
    fill!(rrtstar.all_points, 0)
    rrtstar.num_total_points = 0
    rrtstar.num_improving_points = 0
    fill!(rrtstar.new_batch, 0)
    rrtstar.new_batch[:, 1] .= rrtstar.start
    rrtstar.new_batch_size = 1  # the start point
    rrtstar.num_points_sampled = 1  # the start point
    rrtstar.dist_to_goal_lb = l2dist(rrtstar.start, rrtstar.goal)

    rrtstar.kdtree = KDTree(rrtstar.all_points[:, 1:1])
    rrtstar.viz_r = 0
    rrtstar.viz_new_point = nothing
end


"""Returns the list of nodes, from start node to goal node, on the best solution path if one exists. The logic is to trace the solution path from the goal node to the start node by traversing through the nodes' parents, and reversing the trace before returning."""
function extract_best_path(rrtstar::RRTStarPlanner)::Vector{Point}
    path = Point[]
    if rrtstar.costs[rrtstar.goal] == Inf
        return path
    else
        child = rrtstar.goal
        push!(path, child)
        parent = rrtstar.parents[child]
        while !isnothing(parent)
            child = parent
            push!(path, child)
            parent = rrtstar.parents[child]
        end
    end
    return reverse(path)
end


"""Returns the list of nodes, from goal node to start node (i.e in the reverse order), on the best solution path if one exists. The logic is to trace the solution path from the goal node to the start node by traversing through the nodes' parents."""
function extract_best_path_reversed(rrt::RRTStarPlanner)::Vector{Point}
    path = Point[]
    if rrt.costs[rrt.goal] != Inf
        child = rrt.goal
        push!(path, child)
        parent = rrt.parents[child]
        while !isnothing(parent)
            child = parent
            push!(path, child)
            parent = rrt.parents[child]
        end
    end
    return path
end


"""Runs/resumes the rrtstar search and returns the cost of the best path to the goal node. Assumes that `reset!` function has been invoked already. Once the total number of samples `rrtstar.num_points_sampled` is equal to `rrtstar.max_samples`, the function returns and any subsequent invokations do nothing. The optional argument `sample_limit` (by default `typemax(Int)`) limits the number of samples during this function invokation."""
function search!(rrtstar::RRTStarPlanner; sample_limit::Int = typemax(Int))::Float64
    @assert rrtstar.num_points_sampled > 0 "Looks like reset!(rrtstar) has not been called yet."
    if rrtstar.num_points_sampled >= rrtstar.max_samples
        @error "The search has already ended. `max_samples` number of points have already been sampled."
        return cost(rrtstar)
    end
    for i in 1:sample_limit
        new_point = sample(rrtstar)
        extend(rrtstar, new_point)
        rrtstar.num_points_sampled += 1
        if rrtstar.num_points_sampled >= rrtstar.max_samples
            break
        end
    end
    return cost(rrtstar)
end


