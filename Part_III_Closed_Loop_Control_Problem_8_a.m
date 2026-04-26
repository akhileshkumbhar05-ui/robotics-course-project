clc;
clear;
close all;

%% Known parameters
params.a1 = 1;         % m
params.a2 = 1;         % m
params.g  = 9.8;       % m/s^2

%% True unknown masses (used only in plant simulation)
params.m1_true = 1;    % kg
params.m2_true = 1;    % kg

%% Adaptive control gains
params.Lambda = diag([15 15]);
params.Kd     = diag([120 120]);
params.Gamma  = diag([8 8]);

%% Simulation settings
tspan = [0 10];

% x = [theta1; theta2; dtheta1; dtheta2; m1hat; m2hat]
x0 = [pi/3; pi/3; 0; 1; 0.5; 0.5];

%% Solve ODE
[t, x] = ode23(@(t,x) arm2link_adaptive_ode(t, x, params), tspan, x0);

%% Extract states
theta1  = x(:,1);
theta2  = x(:,2);
dtheta1 = x(:,3);
dtheta2 = x(:,4);
m1hat   = x(:,5);
m2hat   = x(:,6);

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
    q     = [theta1(i); theta2(i)];
    dq    = [dtheta1(i); dtheta2(i)];
    mhat  = [m1hat(i); m2hat(i)];

    qd    = [qd1(i); qd2(i)];
    dqd   = [dqd1(i); dqd2(i)];
    ddqd  = [ddqd1(i); ddqd2(i)];

    e     = q - qd;
    de    = dq - dqd;

    qr_dot  = dqd - params.Lambda*e;
    qr_ddot = ddqd - params.Lambda*de;

    Y = regressor_matrix(q, dq, qr_dot, qr_ddot, params);
    s = de + params.Lambda*e;

    tau = Y*mhat - params.Kd*s;

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
legend('e_1 = \theta_1 - \theta_{d1}', 'e_2 = \theta_2 - \theta_{d2}', 'Location', 'best');

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

%% Optional: estimated masses
figure;
plot(t, m1hat, 'LineWidth', 1.5); hold on;
plot(t, m2hat, 'LineWidth', 1.5);
yline(params.m1_true, '--');
yline(params.m2_true, '--');
grid on;
xlabel('Time (s)');
ylabel('Estimated Mass (kg)');
title('Adaptive Mass Estimates');
legend('\hat{m}_1', '\hat{m}_2', 'm_1 true', 'm_2 true', 'Location', 'best');

%% ================= Local Functions =================

function dx = arm2link_adaptive_ode(t, x, params)
% x = [theta1; theta2; dtheta1; dtheta2; m1hat; m2hat]

    theta1  = x(1);
    theta2  = x(2);
    dtheta1 = x(3);
    dtheta2 = x(4);
    m1hat   = x(5);
    m2hat   = x(6);

    q    = [theta1; theta2];
    dq   = [dtheta1; dtheta2];
    mhat = [m1hat; m2hat];

    %% Desired trajectory
    qd   = [0.1*sin(2*pi*t);
            0.1*cos(2*pi*t)];

    dqd  = [0.2*pi*cos(2*pi*t);
           -0.2*pi*sin(2*pi*t)];

    ddqd = [-0.4*pi^2*sin(2*pi*t);
            -0.4*pi^2*cos(2*pi*t)];

    %% Tracking errors
    e  = q - qd;
    de = dq - dqd;

    %% Reference signals
    qr_dot  = dqd - params.Lambda*e;
    qr_ddot = ddqd - params.Lambda*de;

    %% Filtered error
    s = de + params.Lambda*e;

    %% Regressor
    Y = regressor_matrix(q, dq, qr_dot, qr_ddot, params);

    %% Adaptive control law
    tau = Y*mhat - params.Kd*s;

    %% True plant dynamics
    [M, Cvec, G] = true_robot_dynamics(q, dq, params);
    ddq = M \ (tau - Cvec - G);

    %% Adaptation law
    mhat_dot = -params.Gamma * (Y.') * s;

    %% State derivative
    dx = [dtheta1;
          dtheta2;
          ddq(1);
          ddq(2);
          mhat_dot(1);
          mhat_dot(2)];
end

function Y = regressor_matrix(q, dq, qr_dot, qr_ddot, params)

    theta1  = q(1);
    theta2  = q(2);
    dtheta1 = dq(1);
    dtheta2 = dq(2);

    qr1_dot  = qr_dot(1);
    qr2_dot  = qr_dot(2);
    qr1_ddot = qr_ddot(1);
    qr2_ddot = qr_ddot(2);

    a1 = params.a1;
    a2 = params.a2;
    g  = params.g;

    c1  = cos(theta1);
    c2  = cos(theta2);
    c12 = cos(theta1 + theta2);
    s2  = sin(theta2);

    y11 = a1^2*qr1_ddot + g*a1*c1;

    y12 = (a1^2 + a2^2 + 2*a1*a2*c2)*qr1_ddot ...
        + (a2^2 + a1*a2*c2)*qr2_ddot ...
        - a1*a2*s2*dtheta2*qr1_dot ...
        - a1*a2*s2*(dtheta1 + dtheta2)*qr2_dot ...
        + g*a1*c1 + g*a2*c12;

    y21 = 0;

    y22 = (a2^2 + a1*a2*c2)*qr1_ddot ...
        + a2^2*qr2_ddot ...
        + a1*a2*s2*dtheta1*qr1_dot ...
        + g*a2*c12;

    Y = [y11 y12;
         y21 y22];
end

function [M, Cvec, G] = true_robot_dynamics(q, dq, params)

    theta1  = q(1);
    theta2  = q(2);
    dtheta1 = dq(1);
    dtheta2 = dq(2);

    m1 = params.m1_true;
    m2 = params.m2_true;
    a1 = params.a1;
    a2 = params.a2;
    g  = params.g;

    %% Inertia matrix
    M11 = (m1 + m2)*a1^2 + m2*a2^2 + 2*m2*a1*a2*cos(theta2);
    M12 = m2*a2^2 + m2*a1*a2*cos(theta2);
    M22 = m2*a2^2;

    M = [M11 M12;
         M12 M22];

    %% Coriolis/Centrifugal vector
    h = m2*a1*a2*sin(theta2);

    Cvec = [ -h*(2*dtheta1*dtheta2 + dtheta2^2);
              h*dtheta1^2 ];

    %% Gravity vector
    G = [ (m1 + m2)*g*a1*cos(theta1) + m2*g*a2*cos(theta1 + theta2);
          m2*g*a2*cos(theta1 + theta2) ];
end