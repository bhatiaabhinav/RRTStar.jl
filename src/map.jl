

"""A Set of rectangle obstacles. Note that this function does not ensure that a path from the start node to the goal node exists in the generated map."""
function rand_rect_obstacles(rng, x_range::Tuple{Real,Real}, y_range::Tuple{Real,Real}, start::Tuple{Real,Real}, goal::Tuple{Real,Real}; rect_density::Real = 0.35, rect_sizeratio::Real = 0.03)::Set{AbstractObstacle}
    obstacles::Set{AbstractObstacle} = Set{AbstractObstacle}()

    map_area = (x_range[2] - x_range[1]) * (y_range[2] - y_range[1])
    map_side = mean([x_range[2] - x_range[1], y_range[2] - y_range[1]])
    x_distribution = Uniform(x_range[1], x_range[2])
    y_distribution = Uniform(y_range[1], y_range[2])

    rect_len = rect_sizeratio * map_side
    total_target_area = rect_density * map_area
    total_area = 0.0

    last_box = (0.0, 0.0)
    add_to_cluster = false
    
    while total_area < total_target_area
        bottom_left = (0.0, 0.0)
        if add_to_cluster
            bottom_left = (clamp(rand(rng, Uniform(last_box[1] - rect_len, last_box[1] + 2 * rect_len)), x_range[1], x_range[2]), clamp(rand(rng, Uniform(last_box[2] - rect_len, last_box[2] + 2 * rect_len)), y_range[1], y_range[2]))
        else
            bottom_left = (rand(rng, x_distribution), rand(rng, y_distribution))
        end
        if l2dist(bottom_left, start) > 3 * rect_len && l2dist(bottom_left, goal) > 3 * rect_len
            # "Bidisperse" boxes
            if rand(rng) > 0.5
                push!(obstacles, Rect(bottom_left, 1.4 * rect_len))
                total_area += (1.4 * rect_len)^2
            else
                push!(obstacles, Rect(bottom_left, rect_len))
                total_area += rect_len^2
            end
            # Should the next box be added nearby the most recent box?
            if rand(rng) > 0.5
                add_to_cluster = true
                last_box = bottom_left
            else
                add_to_cluster = false
            end
        end
    end

    return obstacles
end



"""A Set of circular obstacles. Note that this function does not ensure that a path from the start node to the goal node exists in the generated map."""
function rand_circle_obstacles(rng, x_range::Tuple{Real,Real}, y_range::Tuple{Real,Real}, start::Tuple{Real,Real}, goal::Tuple{Real,Real}; circles_density::Real = 0.35, circles_radiusratio::Real = 0.03)::Set{AbstractObstacle}
    obstacles::Set{AbstractObstacle} = Set{AbstractObstacle}()

    map_area = (x_range[2] - x_range[1]) * (y_range[2] - y_range[1])
    map_side = mean([x_range[2] - x_range[1], y_range[2] - y_range[1]])
    x_distribution = Uniform(x_range[1], x_range[2])
    y_distribution = Uniform(y_range[1], y_range[2])

    circle_radius = circles_radiusratio * map_side
    circle_area = π * circle_radius^2
    num_circles = Int(round(circles_density * map_area / (circle_area)))

    num_circles_added = 0
    while num_circles_added < num_circles
        center = (rand(rng, x_distribution), rand(rng, y_distribution))
        if l2dist(center, start) > 2 * circle_radius && l2dist(center, goal) > 2 * circle_radius
            push!(obstacles, Circle(center, circle_radius))
            num_circles_added += 1
        end
    end

    return obstacles
end


"""A Set of line obstacles. Note that this function does not ensure that a path from the start node to the goal node exists in the generated map."""
function rand_line_obstacles(rng, x_range::Tuple{Real,Real}, y_range::Tuple{Real,Real}, start::Tuple{Real,Real}, goal::Tuple{Real,Real}; num_lines::Integer=100, lines_lengthratio::Real = 0.1)::Set{AbstractObstacle}
    obstacles::Set{AbstractObstacle} = Set{AbstractObstacle}()

    map_side = mean([x_range[2] - x_range[1], y_range[2] - y_range[1]])
    x_distribution = Uniform(x_range[1], x_range[2])
    y_distribution = Uniform(y_range[1], y_range[2])

    line_length = lines_lengthratio * map_side

    num_lines_added = 0
    while num_lines_added < num_lines
        center = (rand(rng, x_distribution), rand(rng, y_distribution))
        θ = 2π * rand(rng)
        p1 = center .+ (line_length * cos(θ), line_length * sin(θ)) ./ 2
        p2 = center .- (line_length * cos(θ), line_length * sin(θ)) ./ 2
        if l2dist(center, start) > 2 * line_length && l2dist(center, goal) > 2 * line_length
            push!(obstacles, Line(p1, p2))
            num_lines_added += 1
        end
    end
    return obstacles
end


"""Reads Line obstacles from a map file"""
function load_line_obstacles_from_file(filename::AbstractString)::Set{AbstractObstacle}
    obstacles = Set{AbstractObstacle}()
    open(filename) do f
        for line in readlines(f)
            values = split(line, ',')
            values = parse.(Float64, strip.(values))
            obstacle = Line((values[1], values[2]), (values[3], values[4]))
            push!(obstacles, obstacle)
        end
    end
    return obstacles
end
