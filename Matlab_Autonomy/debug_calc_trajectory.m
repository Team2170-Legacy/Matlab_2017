%
%   debug_calc_trajectory.m
%
%   Script to aid debugging of trajectory generation
%   Martin Krucinski 01/28/2017

f1=figure;
set(f1, 'DefaultLineLineWidth', 3);
h1=stairs(all_t, all_x);
set(h1, 'LineWidth', 3);
grid on							% draw a grid on the figure
ylabel('x [m]')
title('all\_x')

f2=figure;
set(f2, 'DefaultLineLineWidth', 3);
h2=stairs(all_t, all_y);
set(h2, 'LineWidth', 3);
grid on							% draw a grid on the figure
ylabel('y [m]')
title('all\_y')

f3=figure;
set(f3, 'DefaultLineLineWidth', 3);
h3=stairs(all_t, all_theta/deg);
set(h3, 'LineWidth', 3);
grid on							% draw a grid on the figure
ylabel('theta [deg]')
title('all\_theta')

f4 = figure;
axis('equal')					% ensure x & y directions are scale equally on screen
init_Field;
xlim([-1*ft Field.L + 1*ft])					% [m]	set figure limits for x-axis
ylim([-2*ft Field.W + 2*ft])					% [m]	set figure limits for y-axis
%xlim([-30 30])
%ylim([-20 20])
set(f4,'DefaultLineLineWidth',3);	% set figure to draw with thick lines by default
grid on							% draw a grid on the figure
hold on							% ensure multiple drawing commands are overlaid on the figure
                                % without erasing figure first


Field.t = 0;                                
draw_Field(Field);

plot( all_x, all_y, 'b.')
