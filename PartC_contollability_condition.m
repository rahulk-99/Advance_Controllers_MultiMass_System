clear;
clc;
syms t theta1 theta2 F x M m1 m2 l1 l2 g x_dot theta1_dot theta2_dot ;

x_double_dot = (F - m1*l1*sin(theta1)*theta1_dot^2  - m2*l2*sin(theta2)*theta2_dot^2 -m1*g*cos(theta1)*sin(theta1) -m2*g*cos(theta2)*sin(theta2))/(M + m1*(sin(theta1))^2 + m2*(sin(theta2))^2);
theta1_double_dot = (x_double_dot*cos(theta1) - g*sin(theta1))/l1;
theta2_double_dot = (x_double_dot*cos(theta2) - g*sin(theta2))/l2;

state = [x_dot;theta1_dot;theta2_dot;x;theta1;theta2];
input = [F];
state_dot = [x_double_dot;theta1_double_dot;theta2_double_dot;x_dot;theta1_dot;theta2_dot ];

A = jacobian(state_dot,state);
B = jacobian(state_dot,input);
C = eye(6,6);
D = [0;0;0;0;0;0];
x0 = [0;0;0;0;0;0];

Af = subs(A,{x,theta1,theta2,x_dot,theta1_dot,theta2_dot},{0,0,0,0,0,0});
Bf = subs(B,{x,theta1,theta2,x_dot,theta1_dot,theta2_dot},{0,0,0,0,0,0});

controlability_matrix = [Bf Af*Bf Af^2*Bf Af^3*Bf Af^4*Bf Af^5*Bf];
determinant = det(controlability_matrix);

disp("For given system to be controllable,for controllability matrix to have full rank, the following determinant calculated for controlability matrix should not be zero.");
disp(determinant);
disp("Condition :This can happen only when l1 is not equal to l2.");