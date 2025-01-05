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
C = [  0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0;
       0 0 0 1 0 0;
       0 0 0 0 0 0;
       0 0 0 0 0 0];
D = [0;0;0;0;0;0];
x0 = [0;0;0;5;0;0];

Af = subs(A,{x,theta1,theta2,x_dot,theta1_dot,theta2_dot},{0,0,0,0,0,0});
Bf = subs(B,{x,theta1,theta2,x_dot,theta1_dot,theta2_dot},{0,0,0,0,0,0});

An = double(subs(Af,{M,m1,m2,l1,l2,g},{1000,100,100,20,10,9.81}));
Bn = double(subs(Bf,{M,m1,m2,l1,l2,g},{1000,100,100,20,10,9.81}));

Q = 80000*eye(6,6);
R = 0.001;
[K,s,p] = lqr(An,Bn,Q,R);
poles_c = [-0.1,-0.2,-0.3,-0.4,-0.5,-0.6];

Qn = 0.05;
Rn = 0.1;

sim_X = sim("lqg_controller");
time = sim_X.x.Time;
x = sim_X.x.Data;
theta1 = sim_X.theta1.Data;
theta2 = sim_X.theta2.Data;
x_dot = sim_X.x_dot.Data;
theta1_dot = sim_X.theta1_dot.Data;
theta2_dot = sim_X.theta2_dot.Data;

% Plot Position (x) vs Time
figure;
plot(time, x, 'b-', 'LineWidth', 1.5);
title('Position (x) vs Time');
xlabel('Time (seconds)');
ylabel('Position (x)');
grid on;

% Plot Angle (theta1) vs Time
figure;
plot(time, theta1, 'r-', 'LineWidth', 1.5);
title('Angle (theta1) vs Time');
xlabel('Time (seconds)');
ylabel('Angle (theta1)');
grid on;

% Plot Angle (theta2) vs Time
figure;
plot(time, theta2, 'g-', 'LineWidth', 1.5);
title('Angle (theta2) vs Time');
xlabel('Time (seconds)');
ylabel('Angle (theta2)');
grid on;

% Plot Velocity (x_dot) vs Time
figure;
plot(time, x_dot, 'm-', 'LineWidth', 1.5);
title('Velocity (x dot) vs Time');
xlabel('Time (seconds)');
ylabel('Velocity (x dot)');
grid on;

% Plot Angular Velocity (theta1_dot) vs Time
figure;
plot(time, theta1_dot, 'c-', 'LineWidth', 1.5);
title('Angular Velocity (theta1 dot) vs Time');
xlabel('Time (seconds)');
ylabel('Angular Velocity (theta1 dot)');
grid on;

% Plot Angular Velocity (theta2_dot) vs Time
figure;
plot(time, theta2_dot, 'k-', 'LineWidth', 1.5);
title('Angular Velocity (theta2 dot) vs Time');
xlabel('Time (seconds)');
ylabel('Angular Velocity (theta2 dot)');
grid on;



