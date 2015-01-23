
prop_diameter = 0.1397;
rotor_dist = 0.25;
w = rotor_dist+prop_diameter; %width (m)
h = 0.1; %height (m)
d = rotor_dist+prop_diameter; %depth (m)
m = 0.26; %mass (kg)

rotor_to_com = rotor_dist/2;

g = 9.81;

%assume a spherical cow in a vacuum
%or in this case a solid cuboid for the moments of inertia
%I_h = 1/12*m*(w^2+d^2);
I_w = 1/12*m*(h^2+d^2);
I_d = 1/12*m*(h^2+w^2);

state_dims = 10;
input_dims = 3;

A=zeros(state_dims);
A(1:3,4:6) = eye(3);
A(4:6,7:8) = [0,g;-g,0;0,0];
A(7:8,9:10) = eye(2);

B=zeros(state_dims, input_dims);

B(4:6,1) = [0,0,1/m]';
B(9:10,2:3) = diag([rotor_to_com/I_w, rotor_to_com/I_d]);

c = zeros(state_dims,1);

R = diag([1/4,1/2,1/2]);

