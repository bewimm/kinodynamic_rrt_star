

init_quad;

start = [1,1,1, 0,0,0,0,0,0,0];
goal  = [2,2,1, 0,0,0,0,0,0,0];

quad_dim = [0.2,0.2,0.05];

state_limits = ...
    [0,5;
     0,5;
     0,5;
     -5,5;
     -5,5;
     -5,5;
     -1, 1;
     -1, 1;
     -5, 5;
     -5, 5];

 input_limits = ...
     [ -4.545, 9.935;
       -3.62,3.62;
       -3.62,3.62];

 %load obstacles.mat
 obstacles = [];

 disp(['calculating closed form solution']);
 rrt = rrtstar(A,B,c,R);
 disp(['starting algorithm']);

 state_free = @(state, time_range)(is_state_free(state, state_limits, obstacles, quad_dim, time_range));
 input_free = @(input, time_range)(is_input_free(input, input_limits, time_range));
 sample_state = @()(sample_free_states(state_limits, obstacles, quad_dim ));
 display = @(obj, tree, parents, goal_cost, goal_parent)(plot_graph(obj, tree, parents, goal_cost, goal_parent));

 [T, parents] = rrt.run(sample_state, state_free, input_free, start', goal', display);
