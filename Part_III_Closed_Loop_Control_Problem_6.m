clc;
clear;
close all;

%% Parameters
params.m1 = 1;      % kg
params.m2 = 1;      % kg
params.a1 = 1;      % m
params.a2 = 1;      % m
params.g  = 9.8;    % m/s^2

%% Controller gains
params.Kp = diag([2500 2500]);
params.Kv = diag([100 100]);
params.Ki = diag([300 300]);   % integral gains

%% Constant disturbances
params.taud = [5; 5];   % Nm

%% Simulation settings
tspan = [0 10];
x0 = [pi/3; pi/3; 0; 1; 0; 0];   % [theta1; theta2; dtheta1; dtheta2; eta1; eta2]

%% Solve ODE
[t, x] = ode23(@(t,x) arm2link_integral_ctc_ode(t, x, params), tspan, x0);

%% Extract states
theta1  = x(:,1);
theta2  = x(:,2);
dtheta1 = x(:,3);
dtheta2 = x(:,4);
eta1    = x(:,5);
eta2    = x(:,6);

%% Desired trajectories
qd1   = 0.1*sin(2*pi*t);
qd2   = 0.1*cos(2*pi*t);
dqd1  = 0.2*pi*cos(2*pi*t);
dqd2  = -0.2*pi*sin(2*pi*t);
ddqd1 = -0.4*pi^2*sin(2*pi*t);
ddqd2 = -0.4*pi^2*cos(2*pi*t);

%% Tracking errors
e1 = qd1 - theta1;
e2 = qd2 - theta2;

%% Compute control torques for plotting
tau1 = zeros(length(t),1);
tau2 = zeros(length(t),1);

for i = 1:length(t)
    q    = [theta1(i); theta2(i)];
    dq   = [dtheta1(i); dtheta2(i)];
    eta  = [eta1(i); eta2(i)];
    qd   = [qd1(i); qd2(i)];
    dqd  = [dqd1(i); dqd2(i)];
    ddqd = [ddqd1(i); ddqd2(i)];

    e  = qd - q;
    de = dqd - dq;

    [M, Cvec, G] = robot_dynamics(q, dq, params);

    v = ddqd + params.Kv*de + params.Kp*e + params.Ki*eta;
    tau = M*v + Cvec + G;

    tau1(i) = tau(1);
    tau2(i) = tau(2);
end

%% End-effector position
xe = params.a1*cos(theta1) + params.a2*cos(theta1 + theta2);
ye = params.a1*sin(theta1) + params.a2*sin(theta1 + theta2);

%% Plot (a): Time vs joint angles
figure;
plot(t, theta1, 'LineWidth', 1.5); hold on;
plot(t, theta2, 'LineWidth', 1.5);
plot(t, qd1, '--', 'LineWidth', 1.2);
plot(t, qd2, '--', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Joint Angles (rad)');
title('Time vs Joint Angles');
legend('\theta_1', '\theta_2', '\theta_{d1}', '\theta_{d2}', 'Location', 'best');

%% Plot (b): Time vs tracking errors
figure;
plot(t, e1, 'LineWidth', 1.5); hold on;
plot(t, e2, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Tracking Error (rad)');
title('Time vs Tracking Errors');
legend('e_1 = \theta_{d1} - \theta_1', 'e_2 = \theta_{d2} - \theta_2', 'Location', 'best');

%% Plot (c): Time vs control torques
figure;
plot(t, tau1, 'LineWidth', 1.5); hold on;
plot(t, tau2, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Control Torque (N.m)');
title('Time vs Control Torques');
legend('\tau_1', '\tau_2', 'Location', 'best');

%% Plot (d): 3D plot of time vs end-effector position
figure;
plot3(t, xe, ye, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('x_e (m)');
zlabel('y_e (m)');
title('3D Plot of Time vs End-Effector Position');

%% ================= Local Functions =================

function dx = arm2link_integral_ctc_ode(t, x, params)
% x = [theta1; theta2; dtheta1; dtheta2; eta1; eta2]

    theta1  = x(1);
    theta2  = x(2);
    dtheta1 = x(3);
    dtheta2 = x(4);
    eta1    = x(5);
    eta2    = x(6);

    q   = [theta1; theta2];
    dq  = [dtheta1; dtheta2];
    eta = [eta1; eta2];

    %% Desired trajectory
    qd   = [0.1*sin(2*pi*t);
            0.1*cos(2*pi*t)];

    dqd  = [0.2*pi*cos(2*pi*t);
           -0.2*pi*sin(2*pi*t)];

    ddqd = [-0.4*pi^2*sin(2*pi*t);
            -0.4*pi^2*cos(2*pi*t)];

    %% Errors
    e  = qd - q;
    de = dqd - dq;

    %% Robot dynamics
    [M, Cvec, G] = robot_dynamics(q, dq, params);

    %% Integral computed-torque controller
    v = ddqd + params.Kv*de + params.Kp*e + params.Ki*eta;
    tau = M*v + Cvec + G;

    %% Disturbed plant
    ddq = M \ (tau + params.taud - Cvec - G);

    %% Integral error dynamics
    deta = e;

    %% State derivative
    dx = [dtheta1;
          dtheta2;
          ddq(1);
          ddq(2);
          deta(1);
          deta(2)];
end

function [M, Cvec, G] = robot_dynamics(q, dq, params)

    theta1  = q(1);
    theta2  = q(2);
    dtheta1 = dq(1);
    dtheta2 = dq(2);

    m1 = params.m1;
    m2 = params.m2;
    a1 = params.a1;
    a2 = params.a2;
    g  = params.g;

    %% Inertia matrix
    M11 = (m1 + m2)*a1^2 + m2*a2^2 + 2*m2*a1*a2*cos(theta2);
    M12 = m2*a2^2 + m2*a1*a2*cos(theta2);
    M21 = M12;
    M22 = m2*a2^2;

    M = [M11 M12;
         M21 M22];

    %% Coriolis/Centrifugal vector
    h = m2*a1*a2*sin(theta2);

    Cvec = [ -h*(2*dtheta1*dtheta2 + dtheta2^2);
              h*dtheta1^2 ];

    %% Gravity vector
    G = [ (m1 + m2)*g*a1*cos(theta1) + m2*g*a2*cos(theta1 + theta2);
          m2*g*a2*cos(theta1 + theta2) ];
end