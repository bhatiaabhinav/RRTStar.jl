import Luxor as L

SCREEN_SIZE = (1000, 1000)

function visualize_init(rrtstar::RRTStarPlanner; filename="tmp.svg", size=(1000, 1000))
    global SCREEN_SIZE = size
    L.Drawing(SCREEN_SIZE[1], SCREEN_SIZE[2], filename)
end

function visualize_frame(rrtstar::RRTStarPlanner; return_argb=true)
    L.origin(L.Point(SCREEN_SIZE .÷ 2))
    L.background("white")
    draw_obstacles(rrtstar)
    draw_tree_lines(rrtstar, rrtstar.start, rrtstar.children)
    draw_tree_nodes(rrtstar, rrtstar.start, rrtstar.children)
    draw_solution(rrtstar, rrtstar.goal, rrtstar.parents)
    draw_start_goal_nodes(rrtstar)
    rrtstar.focus_level > 0 && draw_focus_area(rrtstar)
    mat = return_argb ? L.image_as_matrix() : nothing
    return mat
end

function visualize_finish(rrtstar::RRTStarPlanner)
    L.finish()
end

function visualize(rrtstar::RRTStarPlanner, filename; size=(1000, 1000))
    visualize_init(rrtstar, filename=filename, size=size)
    visualize_frame(rrtstar)
    visualize_finish(rrtstar)
end

function toscreen(rrt::RRTStarPlanner, p::Point)
    L.Point(SCREEN_SIZE .* p ./ (rrt.x_range[2] - rrt.x_range[1], rrt.y_range[2] - rrt.y_range[1]))
end

function draw(rrt::RRTStarPlanner, l::Line)
    L.setline(1)
    L.line(toscreen(rrt, l.p1), toscreen(rrt, l.p2), :stroke)
end

function draw(rrt::RRTStarPlanner, r::Rect)
    topleft = toscreen(rrt, r.topleft)
    sides = toscreen(rrt, r.size)
    L.rect(topleft.x, topleft.y, sides.x, sides.y, :fill)
end

function draw(rrt::RRTStarPlanner, c::Circle)
    radii = toscreen(rrt, (c.radius, c.radius))
    L.ellipse(toscreen(rrt, c.center), radii.x, radii.y, :fill)
end

function draw_obstacles(rrt::RRTStarPlanner)
    L.setcolor("black")
    foreach(rrt.obstacles) do ob
        draw(rrt, ob)
    end
end

function draw_start_goal_nodes(rrt::RRTStarPlanner)
    L.setcolor("red1")
    L.circle(toscreen(rrt, rrt.start), 15, :fill)
    L.setcolor("green1")
    L.circle(toscreen(rrt, rrt.goal), 15, :fill)
end

function draw_tree_nodes(rrt::RRTStarPlanner, parent, children)
    L.setcolor("cyan")
    for child in children[parent]
        L.circle(toscreen(rrt, child), 4, :fill)
        draw_tree_nodes(rrt, child, children)
    end
end

function draw_tree_lines(rrt::RRTStarPlanner, parent, children)
    L.setcolor("blue")
    L.setline(4)
    for child in children[parent]
        L.line(toscreen(rrt, parent), toscreen(rrt, child), :stroke)
        draw_tree_lines(rrt, child, children)
    end
end

function draw_solution(rrt::RRTStarPlanner, goal, parents)
    L.setcolor("magenta")
    L.setline(10)
    if haskey(parents, goal)
        child = goal
        parent = parents[child]
        while parent !== nothing
            L.line(toscreen(rrt, child), toscreen(rrt, parent), :stroke)
            child = parent
            parent = parents[child]
        end
    end
end

function draw_focus_area(rrt::RRTStarPlanner)
    L.setcolor("darkorange1")
    L.setline(8)
    top_left = (rrt.focus_index .- 1) .* (SCREEN_SIZE .÷ 4) .- SCREEN_SIZE ./ 2
    L.rect(top_left[1], top_left[2], SCREEN_SIZE[1] ÷ 4, SCREEN_SIZE[2] ÷ 4, :stroke)
end

