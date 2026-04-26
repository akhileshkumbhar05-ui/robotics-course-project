clc;
clear;
close all;

params.m1 = 1;
params.m2 = 1;
params.a1 = 1;
params.a2 = 1;
params.g  = 9.8;

tspan = [0 20];
x0 = [0; 0; 0; 0];

[t, x] = ode23(@(t,x) arm2link_openloop_ode(t, x, params), tspan, x0);

theta1  = x(:,1);
theta2  = x(:,2);
dtheta1 = x(:,3);
dtheta2 = x(:,4);

xe = params.a1*cos(theta1) + params.a2*cos(theta1 + theta2);
ye = params.a1*sin(theta1) + params.a2*sin(theta1 + theta2);

figure;
plot(t, theta1, 'LineWidth', 1.5); hold on;
plot(t, theta2, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Joint Angles (rad)');
title('Time vs Joint Angles');
legend('\theta_1','\theta_2');

figure;
plot(t, dtheta1, 'LineWidth', 1.5); hold on;
plot(t, dtheta2, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Joint Angular Velocity (rad/s)');
title('Time vs Joint Angular Velocities');
legend('\omega_1','\omega_2');

figure;
plot3(t, xe, ye, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('x_e (m)');
zlabel('y_e (m)');
title('3D Plot of Time vs End-Effector Position');

function dx = arm2link_openloop_ode(t, x, params)
    theta1  = x(1);
    theta2  = x(2);
    dtheta1 = x(3);
    dtheta2 = x(4);

    m1 = params.m1;
    m2 = params.m2;
    a1 = params.a1;
    a2 = params.a2;
    g  = params.g;

    tau = [sin(2*pi*t); cos(2*pi*t)];

    M11 = (m1 + m2)*a1^2 + m2*a2^2 + 2*m2*a1*a2*cos(theta2);
    M12 = m2*a2^2 + m2*a1*a2*cos(theta2);
    M22 = m2*a2^2;

    M = [M11 M12;
         M12 M22];

    h = m2*a1*a2*sin(theta2);

    N1 = -h*(2*dtheta1*dtheta2 + dtheta2^2) ...
         + (m1 + m2)*g*a1*cos(theta1) ...
         + m2*g*a2*cos(theta1 + theta2);

    N2 = h*dtheta1^2 + m2*g*a2*cos(theta1 + theta2);

    N = [N1; N2];

    ddq = M \ (tau - N);

    dx = [dtheta1;
          dtheta2;
          ddq(1);
          ddq(2)];
end