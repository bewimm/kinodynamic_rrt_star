# About
This is an implementation of parts of the paper:  
Kinodynamic RRT*: Optimal Motion Planning for Systems with Linear Differential Constraints  
by Dustin J. Webb and Jur van den Berg  
[PDF](arxiv.org/pdf/1205.5088)

If you use the code then I'm sure the authors would appreciate a citation:
```
@article{DBLP:journals/corr/abs-1205-5088,
    author    = {Dustin J. Webb and Jur van den Berg},
    title     = {Kinodynamic RRT*: Optimal Motion Planning for Systems with Linear Differential Constraints},
    journal   = {CoRR},
    volume    = {abs/1205.5088},
    year      = {2012},
    url       = { http://arxiv.org/abs/1205.5088 },
}
```
    
# Limitations
* This implementation does not (currently) solve systems where the matrix A is not nilpotent (see the paper for details).
* The search radius is not decreased with each iteration (see section V) 

# Demos
## A 2D double integrator
run it using `rrt_double_int_2d.m`
## A quadcopter model with obstacles
use `rrt_quad_test.m` to run it  
waypoints are currently not used (only displayed)  
obstacles are rows of 6 values:  
`(center_x, center_y, center_z, width, length, height)`  
e.g.
```
obstacles = [-1,20,-2,   0.5,4,4;
             -3,15,-3.5, 0.5,6,1];
```