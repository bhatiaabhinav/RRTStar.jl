# RRT* 

Julia implementation of RRT* motion planning algorithm. Only 2D path planning is supported yet.


<center><img src="https://github.com/bhatiaabhinav/RRTStar.jl/blob/main/rrtstar.gif" width="300"/></center>

## Installation

```julia
using Pkg
Pkg.add("https://github.com/bhatiaabhinav/RRTStar.jl")
```
## Examples

### Generating a map and solving it using RRT*

```julia
using Random
using FileIO
using RRTStar

x_range = (-5.0, 5.0)       # Map size
y_range = (-5.0, 5.0)       # Map size
start = (-4.8, 4.8)         # Start point
goal = (4.8, -4.8)          # Goal point
max_samples = 5000         # budget of samples for RRT* execution
rng = MersenneTwister(0)    # A random number generator with seed 0

obstacles_rect = rand_rect_obstacles(rng, x_range, y_range, start, goal)  # Generate a `Set` of rectanglur obstacles. Optional arguments rect_density and rect_sizeratio control number and size of obstacles. See src/map.jl
obstacles_circ = rand_circle_obstacles(rng, x_range, y_range, start, goal)  # Generate a set of circlur obstacles. Optional arguments circles_density and circles_radiusratio control number and size of obstacles. See src/map.jl
obstacles_line = rand_line_obstacles(rng, x_range, y_range, start, goal)  # Generate a set of line-segment obstacles. Optional arguments num_lines and lines_lengthratio control number and length of obstacles. See src/map.jl

obstacles = union(obstacles_rect, obstacles_line)  # Since obstacles_rect, obstacles_circ and obstacles_line are julia sets, they can be combined. Each obstacle type `Rect`, `Circle` and `Line` inherits `AbstractObstacle`. New obstacle types can be defined by inheriting AbstractObstacle and implementing intersection detection function `intersects(line::Line, obs::NewObstacleType)::Bool`. See src/obstacle.jl

rrtstar = RRTStarPlanner(x_range, y_range, start, goal, obstacles, max_samples)  # create RRT* planner. see src/rrtstar.jl for optional arguments e.g., the growth factor hyperparameter.

Random.seed!(rrtstar, 0)    # Optionally seed the planner for deterministic results
reset!(rrtstar)             # Needs to called before search
search!(rrtstar)            # Run RRT* until max_sample samples
println(cost(rrtstar))      # Print the cost of the best solution
visualize(rrtstar, "rrtstar.svg", size=(1000, 1000))    # Visualize the final state of tree. Saving with png extension is also supported. Optional argument `size` specifies image size. (1000, 1000) by default. To visualize custom obstacle types, implement function `draw(rrt::RRTStarPlanner, obs::NewObstacleType)`, which should use Luxor.jl package. See src/visualization.jl

```

### Recording an animation

An animation can be created by running RRT* incrementally and visualizing intermediate frames.

```julia
rrtstar = RRTStarPlanner(x_range, y_range, start, goal, obstacles, max_samples)
Random.seed!(rrtstar, 0)
reset!(rrtstar)

frames = []     # An array to hold frames of the animation
visualize_init(rrtstar, size=(1000, 1000))  # initialize. Optional argument `size` specifies image size. (1000, 1000) by default.
while rrtstar.num_points_sampled < rrtstar.max_samples
    search!(rrtstar, sample_limit=100)     # Run RRT* for for 100 samples only in this iteration of the while loop.
    push!(frames, visualize_frame(rrtstar)) # Visualize current state of the tree and append to the frames array
    println("samples: ", rrtstar.num_points_sampled, "\t best path cost: ",  cost(rrtstar))
end

save("rrtstar.gif", cat(frames..., dims=3), fps=20)  # Concatentate the frames into a 3D array and save as a gif.
```


