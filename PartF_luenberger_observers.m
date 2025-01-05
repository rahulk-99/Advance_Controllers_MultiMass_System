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

Af = subs(A,{x,theta1,theta2,x_dot,theta1_dot,theta2_dot},{0,0,0,0,0,0});
Bf = subs(B,{x,theta1,theta2,x_dot,theta1_dot,theta2_dot},{0,0,0,0,0,0});

An = double(subs(Af,{M,m1,m2,l1,l2,g},{1000,100,100,20,10,9.81}));
Bn = double(subs(Bf,{M,m1,m2,l1,l2,g},{1000,100,100,20,10,9.81}));


% Observer for x(t)
C1 = [ 0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 1 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0];
poles_c1 = [-0.1,-0.2,-0.3,-0.4,-0.5,-0.6];
L1_T = place(An',C1',poles_c1);
L1 = L1_T';
disp("Observer gain matrix for vector x(t)");
disp(L1);


% observability check x(t),theta2(t)
C3 = [ 0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 1 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 1];
poles_c3 = [-0.1,-0.2,-0.3,-0.4,-0.5,-0.6];
L3_T = place(An',C3',poles_c3);
L3 = L3_T';
disp("Observer gain matrix for vector x(t),theta2(t)");
disp(L3);

%observability check x(t),theta1(t),theta2(t)
C4 = [ 0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 1 0 0;
       0 0 0 0 1 0;
       0 0 0 0 0 1];
poles_c4 = [-0.1,-0.2,-0.3,-0.4,-0.5,-0.6];
L4_T = place(An',C4',poles_c4);
L4 = L4_T';
disp("Observer gain matrix for  vector x(t),theta1(t),theta2(t)");
disp(L4);
    