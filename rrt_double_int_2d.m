
start = [2,2,0,0];
goal  = [98,98,0,0];

radius = 1;

state_limits = ...
    [0,100;
    0,100;
    -10,10;
    -10,10];

input_limits = ...
    [ -5, 5;
    -5, 5];

state_dims = 4;
input_dims = 2;

A=zeros(state_dims);
A(1,3) = 1;
A(2,4) = 1;


B=zeros(state_dims, input_dims);
B(3,1) = 1;
B(4,2) = 1;

c = zeros(state_dims,1);

R = eye(2);

%obstacles = [40,10,20,80];
obstacles = [60,0,10,20;
             60,30,10,70;
             30,0,10,70;
             30,80,10,20];

disp(['calculating closed form solution']);
rrt = rrtstar(A,B,c,R);
disp(['starting algorithm']);

state_free = @(state, time_range)(ball_is_state_free(state, state_limits, obstacles, radius, time_range));
input_free = @(input, time_range)(is_input_free(input, input_limits, time_range));
sample_state = @()(ball_sample_free_states(state_limits, obstacles, radius ));
display = @(scratch, obj, tree, parents, goal, goal_cost, goal_parent)(ball_plot_field(scratch, obj, tree, parents, obstacles, goal, goal_cost, goal_parent));


[T, parents] = rrt.run(sample_state, state_free, input_free, start', goal', display);
