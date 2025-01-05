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
D = [0;0;0;0;0;0];
x0 = [0;0;0;0;0;0];

Af = subs(A,{x,theta1,theta2,x_dot,theta1_dot,theta2_dot,M,m1,m2,l1,l2,g},{0,0,0,0,0,0,1000,100,100,20,10,9.81});
Bf = subs(B,{x,theta1,theta2,x_dot,theta1_dot,theta2_dot,M,m1,m2,l1,l2,g},{0,0,0,0,0,0,1000,100,100,20,10,9.81});

% observability check x(t)
C1 = [ 0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 1 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0];
observability_matrixC1 = [C1 ;C1*Af; C1*Af^2; C1*Af^3; C1*Af^4 ;C1*Af^5];
disp("Matrix C1:");
disp(C1);
disp("Observability matrix:");
disp(observability_matrixC1);
disp("Rank for observability matrix:");
disp(rank(observability_matrixC1));

% observability check theta1(t),theta2(t)
C2 = [ 0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 0 1 0;
       0 0 0 0 0 1];
observability_matrixC2 = [C2 ;C2*Af; C2*Af^2; C2*Af^3; C2*Af^4 ;C2*Af^5];
disp("Matrix C2:");
disp(C2);
disp("Observability matrix:");
disp(observability_matrixC2);
disp("Rank for observability matrix:");
disp(rank(observability_matrixC2));

% observability check x(t),theta2(t)

C3 = [ 0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 1 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 1];
observability_matrixC3 = [C3 ;C3*Af; C3*Af^2; C3*Af^3; C3*Af^4 ;C3*Af^5];
disp("Matrix C3:");
disp(C3);
disp("Observability matrix:");
disp(observability_matrixC3);
disp("Rank for observability matrix:");
disp(rank(observability_matrixC3));

%observability check x(t),theta1(t),theta2(t)

C4 = [ 0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 1 0 0;
       0 0 0 0 1 0;
       0 0 0 0 0 1];
observability_matrixC4 = [C4 ;C4*Af; C4*Af^2; C4*Af^3; C4*Af^4 ;C4*Af^5];
disp("Matrix C4:");
disp(C4);
disp("Observability matrix:");
disp(observability_matrixC4);
disp("Rank for observability matrix:");
disp(rank(observability_matrixC4));