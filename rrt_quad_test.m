

init_quad;

start = [0,0,2, 0,0,0,0,0,0,0];
goal  = [-3,20,2, 0,0,0,0,0,0,0];

quad_radius = max([w,h,d]);

state_limits = ...
    [-1e5,1e5;   %pos x
     -1e5,1e5;   %pos y
     0,1e5;      %pos z
     -1e5, 1e5;  %vel x
     -1e5, 1e5;  %vel y
     -1e5, 1e5;  %vel z
     -1e5, 1e5;  %pitch
     -1e5, 1e5;  %roll
     -1e5, 1e5;  %vel pitch
     -1e5, 1e5]; %vel roll

 sampling_limits = ...
     [-10,10; %pos x
     0,30;   %pos y
     0,4;    %pos z
     -5,5;   %vel x
     -5,5;   %vel y
     -5,5;   %vel z
     -1, 1;  %pitch
     -1, 1;  %roll
     -5, 5;  %vel pitch
     -5, 5]; %vel roll
 
 input_limits = ...
     [ -4.545, 9.935;
       -3.62,3.62;
       -3.62,3.62];

 load obstacles.mat
 %obstacles = [];
 load waypoints.mat
 
% obstacles(:,[1:3]) = obstacles(:,[4:6]);
obstacles(:,3) = -obstacles(:,3); %-z in the data is up
waypoints(:,3) = -waypoints(:,3);
 
 disp(['calculating closed form solution']);
 rrt = rrtstar(A,B,c,R);
 disp(['starting algorithm']);

 state_free = @(state, time_range)(quad_is_state_free(state, state_limits, obstacles, quad_radius, time_range));
 input_free = @(input, time_range)(is_input_free(input, input_limits, time_range));
 sample_state = @()(quad_sample_free_states(sampling_limits,state_limits, obstacles, quad_radius ));
 display = @(scratch, obj, tree, parents, goal_cost, goal_parent)(quad_plot_field(scratch, obj, tree, parents, obstacles, waypoints, goal', goal_cost, goal_parent));

 [T, parents] = rrt.run(sample_state, state_free, input_free, start', goal', display);
