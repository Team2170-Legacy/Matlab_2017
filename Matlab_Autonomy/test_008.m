%
%   test_008.m
%
%   Test of robot trajectory generation
%
%   Martin Krucinski

init_Constants;

%*** NOT NEEDED, init_Robot_v002 now calls init_Field_v002
%**** init_Field_v002;

init_Robot_v002;

R       = Robot.R;
d       = Robot.d;
L       = Robot.L;


% x0 = Robot.x0;
% y0 = Robot.y0;
% theta0 = Robot.theta0;

% xf      = 1;
% yf      = 1;
% thetaf  = -135*deg;


vx0     = 0;
vy0     = 0;
omega0  = 0;
vxf     = 0;
vyf     = 0;
omegaf  = 0;

tfinal      =  2.0;

Ts          = Robot.Ts;

%start_pos   = 'RS1';    end_pos     = 'RP1';
%start_pos   = 'BS3';    end_pos     = 'BP3';
%start_pos   = 'RS3';    end_pos     = 'RP3';
%start_pos   = 'BS1';    end_pos     = 'BP1';
%start_pos   = 'RS2';    end_pos     = 'RP2';
start_pos   = 'BS2';    end_pos     = 'BP2';


[all_omega_R, all_omega_L, all_t,t_auto_end, i_auto_end] = calc_trajectory_v8(start_pos,end_pos,vx0,vy0,omega0,vxf,vyf,omegaf, Robot, Field, Ts);
%   Calculate trajectories

Robot.Start_Pos     = eval([ 'Field.' start_pos ]);        % *** NOTE! *** This has to match the starting position for calc_trajectory_v8 ***

t_final     = all_t(end);

make_dot_h_file_v002(start_pos,end_pos,all_omega_R,all_omega_L,all_t,Robot,t_auto_end, i_auto_end);

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

