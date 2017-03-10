%
%   test_005.m
%
%   Test of robot trajectory generation
%
%   Martin Krucinski

init_Constants;
init_Field;
init_Robot;

R       = Robot.R;
d       = Robot.d;
L       = Robot.L;

% x0      = 0.0 + L/2;
% y0      = 7;
% theta0  = 0*deg;
x0 = Robot.x0;
y0 = Robot.y0;
theta0 = Robot.theta0;

xf      = 1;
yf      = 1;
thetaf  = -135*deg;
vx0     = 0;
vy0     = 0;
omega0  = 0;
vxf     = 0;
vyf     = 0;
omegaf  = 0;

tfinal      =  2.0;

Ts          = Robot.Ts;
[all_omega_R, all_omega_L, all_t] = calc_trajectory_v5(x0,y0,theta0,xf,yf,thetaf,vx0,vy0,omega0,vxf,vyf,omegaf, Robot, Ts);
%   Calculate trajectories

t_final     = all_t(end);


% %%
% f1=figure;
% title('Robot Trajectory')
% set(gcf, 'DefaultLineLineWidth', 3);
% 
% subplot(311)
% plot(all_t, all_x)
% ylabel('x  [m]')
% grid on
% hold on
% plot(0, x0, 'bo')
% plot(tfinal, xf, 'bo')
% 
% subplot(312)
% set(gca, 'DefaultLineLineWidth', 3);
% plot(all_t, all_y)
% ylabel('y [m]')
% grid on
% hold on
% plot(0, y0, 'bo')
% plot(tfinal, yf, 'bo')
% 
% subplot(313)
% plot(all_t, all_theta)
% ylabel('theta [rad]')
% xlabel('t [s]')
% grid on
% hold on
% plot(0, theta0, 'bo')
% plot(tfinal, thetaf, 'bo')
% 
% %%
% f2=figure;
% title('Robot Velocities')
% set(gcf, 'DefaultLineLineWidth', 3);
% 
% subplot(311)
% plot(all_t, all_vx)
% ylabel('vx  [m/s]')
% grid on
% hold on
% plot(0, vx0, 'bo')
% plot(tfinal, vxf, 'bo')
% 
% subplot(312)
% set(gca, 'DefaultLineLineWidth', 3);
% plot(all_t, all_vy)
% ylabel('vy [m/s]')
% grid on
% hold on
% plot(0, vy0, 'bo')
% plot(tfinal, vyf, 'bo')
% 
% subplot(313)
% plot(all_t, all_omega)
% ylabel('omega [rad/s]')
% grid on
% hold on
% plot(0, omega0, 'bo')
% plot(tfinal, omegaf, 'bo')
% xlabel('t [s]')
% 
% 
% %   Now, ideally we need polynomials for vR & vL
% %   For now, just calculate the velocity points
% 
% all_v_Fwd       = sqrt(all_vx.^2+all_vy.^2);
% 
% all_v_R         = all_v_Fwd + 1/2 * all_omega * d;
% all_v_L         = all_v_Fwd - 1/2 * all_omega * d;
% all_omega_R     = all_v_R / R;
% all_omega_L     = all_v_L / R;

% %%
% f3=figure;
% title('Wheel Linear Velocities')
% set(gcf, 'DefaultLineLineWidth', 3);
% 
% subplot(211)
% plot(all_t, all_v_R)
% ylabel('Right Wheel Linear Velocity [m/s]')
% grid on
% 
% subplot(212)
% plot(all_t, all_vx)
% plot(all_t, all_v_L)
% ylabel('Left Wheel Linear Velocity [m/s]')
% xlabel('t [s]')
% grid on

%%
f3=figure;
title('Wheel Angular Velocities')
set(gcf, 'DefaultLineLineWidth', 3);

subplot(211)
plot(all_t, all_omega_R,'r')
ylabel('Right Wheel Velocity [rad/s]')
grid on

subplot(212)
plot(all_t, all_omega_L,'b')
ylabel('Left Wheel Velocity [rad/s]')
xlabel('t [s]')
grid on

