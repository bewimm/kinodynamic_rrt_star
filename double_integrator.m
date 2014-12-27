
A = [0,1;0,0];
B = [0;1];
c = [0;0];
R = 1;

x0 = [0;0];
x1 = [1;1];

rrt = rrtstar(A,B,c,R);

%%change factor in rrt to 4 before using
cost = rrt.evaluate_cost(x0,x1);
