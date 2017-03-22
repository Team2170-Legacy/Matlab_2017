%
%   plot_vision.m
%
%   Martin Krucinski
%   2017-03-18
%
%   Script to plot simulation variables from robot simulation with vision
%   feedback for steering

f1=figure;
%set(f1, 'DefaultLineLineWidth', 3);
he = stairs(all_t, Robot.e_Gear_x_all, 'b');
W    = 3;
set(he,'LineWidth',W);
grid on
xlabel('t [s]')
ylabel('error [pixels]')
title([ 'e\_Gear\_x for   fps = ' num2str(fps) '   Kp = ' num2str(Kp) '   delay = ' num2str(camera_delay)])

% f2=figure;
% %set(f1, 'DefaultLineLineWidth', 3);
% hd = stairs(all_t, Robot.target_distance_all, 'b');
% W    = 3;
% set(hd,'LineWidth',W);
% grid on
% xlabel('t [s]')
% ylabel('distance [m]')
% title('target\_distance\_all')


