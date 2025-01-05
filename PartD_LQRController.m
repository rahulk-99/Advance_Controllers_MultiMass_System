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
x0 = [0;0;0;5;0;0];

Af = subs(A,{x,theta1,theta2,x_dot,theta1_dot,theta2_dot},{0,0,0,0,0,0});
Bf = subs(B,{x,theta1,theta2,x_dot,theta1_dot,theta2_dot},{0,0,0,0,0,0});

An = double(subs(Af,{M,m1,m2,l1,l2,g},{1000,100,100,20,10,9.81}));
Bn = double(subs(Bf,{M,m1,m2,l1,l2,g},{1000,100,100,20,10,9.81}));

rank_of_matix = rank(ctrb(An,Bn));

disp("The rank of controllability matrix after substitution is:");
disp(rank_of_matix);

Q = 850000*eye(6,6);
R = 0.0001;
[K,s,p] = lqr(An,Bn,Q,R);

disp("Choosen Q:");
disp(Q);

disp("Choosen R:");
disp(R);

disp("Obtained K:");
disp(K);

disp("Lyapunov indirect method for stability check:");
disp("Eigen values of closed loop A for linearized system");
disp(eig(An - Bn*K));

sim_X = sim("lqr_controller");

time = sim_X.linear_x.Time;
x = sim_X.linear_x.Data;  % State variable 'x'
theta1 = sim_X.linear_theta1.Data;  % State variable 'theta1'
theta2 = sim_X.linear_theta2.Data;  % State variable 'theta2'
x_dot = sim_X.linear_xdot.Data;  % State variable 'x_dot'
theta1_dot = sim_X.linear_theta1_dot.Data;  % State variable 'theta1_dot'
theta2_dot = sim_X.linear_theta2_dot.Data;  % State variable 'theta2_dot'

nx = sim_X.nonlinear_x1.Data;
theta1_non_linear = sim_X.nonlinear_theta1.Data;  % State variable 'theta1'
theta2_non_linear = sim_X.nonlinear_theta2.Data;  % State variable 'theta2'
x_dot_non_linear = sim_X.nonlinear_xdot1.Data;  % State variable 'x_dot'
theta1_dot_non_linear = sim_X.nonlinear_theta1_dot1.Data;  % State variable 'theta1_dot'
theta2_dot_non_linear = sim_X.nonlinear_theta2_dot1.Data;  % State variable 'theta2_dot'


% Position (x) vs Time
figure;1.5
title('Linear model') ;
plot(time, x, 'b-', 'LineWidth', 0.5, 'DisplayName', 'x (linear)');  % Linear model
hold on;
plot(time, nx, 'r-', 'LineWidth', 0.5, 'DisplayName', 'nx (non-linear)');  % Non-linear model
hold off;
title('Position (x) vs Time');
xlabel('Time (seconds)');
ylabel('x (position)');
legend('show');
grid on;

% Angle (theta1) vs Time
figure;
plot(time, theta1, 'b-', 'LineWidth', 0.5, 'DisplayName', 'theta1 (linear)');
hold on;
plot(time, theta1_non_linear, 'r-', 'LineWidth', 0.5, 'DisplayName', 'theta1 (non-linear)');
hold off;
title('Angle (theta1) vs Time');
xlabel('Time (seconds)');
ylabel('theta1 (angle)');
legend('show');
grid on;

% Angle (theta2) vs Time
figure;
plot(time, theta2, 'b-', 'LineWidth', 0.5, 'DisplayName', 'theta2 (linear)');
hold on;
plot(time, theta2_non_linear, 'r-', 'LineWidth', 0.5, 'DisplayName', 'theta2 (non-linear)');
hold off;
title('Angle (theta2) vs Time');
xlabel('Time (seconds)');
ylabel('theta2 (angle)');
legend('show');
grid on;

% Velocity (x_dot) vs Time
figure;
plot(time, x_dot, 'b-', 'LineWidth', 0.5, 'DisplayName', 'x dot (linear)');
hold on;
plot(time, x_dot_non_linear, 'r-', 'LineWidth', 0.5, 'DisplayName', 'x dot (non-linear)');
hold off;
title('Velocity (x dot) vs Time');
xlabel('Time (seconds)');
ylabel('x dot (velocity)');
legend('show');
grid on;

% Angular Velocity (theta1_dot) vs Time
figure;
plot(time, theta1_dot, 'b-', 'LineWidth', 0.5, 'DisplayName', 'theta1 dot (linear)');
hold on;
plot(time, theta1_dot_non_linear, 'r-', 'LineWidth', 0.5, 'DisplayName', 'theta1 dot (non-linear)');
hold off;
title('Angular Velocity (theta1 dot) vs Time');
xlabel('Time (seconds)');
ylabel('theta1 dot (angular velocity)');
legend('show');
grid on;

% Angular Velocity (theta2_dot) vs Time
figure;
plot(time, theta2_dot, 'b-', 'LineWidth', 0.5, 'DisplayName', 'theta2 dot (linear)');
hold on;
plot(time, theta2_dot_non_linear, 'r-', 'LineWidth', 0.5, 'DisplayName', 'theta2 dot (non-linear)');
hold off;
title('Angular Velocity (theta2 dot) vs Time');
xlabel('Time (seconds)');
ylabel('theta2 dot (angular velocity)');
legend('show');
grid on;