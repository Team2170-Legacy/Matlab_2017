%
%   init_Robot_v002.m
%
%   Jacob Krucinski
%
%   03/02/2017  Updated Martin Krucinski to re-arranged field starting
%   positions

init_Constants

%% Simulation sampling time
Robot.Ts        = 0.010;        % [s]

%% Robot dimensions and velocity
Robot.d	= 21*in;		% distance between wheel centerlines
Robot.R = 2*in;         % [m]		Robot wheel radius
Robot.L = 22*in;		% Robot length (along the driving direction)
Robot.W = 24.5*in;
%
%   Update Martin Krucinski 03/03/2017
%   Per info on our robot, max speed is 12.6 fps, which is 3.84 m/s
%   We use v_max = 3.048 m/s below, 12.6 fps * 80% is 3.07 m/s.
%   With the additional margin = 0.80 in the calc_*_* scripts
%   max robot velocity will be 2.458 m/s, hopefully sufficient margin.
%   0.80 * 0.80 = 0.64, i.e. 64% of max velocity

%Robot.v_max = 3.048;    % Robot max velocity (on each R / L drive wheel)
Robot.v_max = 2.000;    % Robot max velocity (on each R / L drive wheel)
Robot.a_max = 3;        %Robot assumed acceleration

%init_Field_v002;
init_Field_v004;

% % Robot starting position selection
% 
% Robot.Start_Pos     = Field.RS3;    % CHANGE SPECIFIED STARTING POSITION HERE
%                                     % OR
%                                     % in the selection of Autonomous
%                                     % Trajectory, calc_trajectory_v8.m and
%                                     % its associated helper scripts, e.g.
%                                     % calc_RS3_RB_v001.m etc.
% 
% Robot.x0 = Robot.Start_Pos.x;
% Robot.y0 = Robot.Start_Pos.y;
% Robot.theta0 = Robot.Start_Pos.theta;

%   Prepare figure for field drawing
f1		= figure;				% open figure
axis('equal')					% ensure x & y directions are scale equally on screen
xlim([-1*ft Field.L + 1*ft])					% [m]	set figure limits for x-axis
ylim([-2*ft Field.W + 2*ft])					% [m]	set figure limits for y-axis
%xlim([-30 30])
%ylim([-20 20])
set(f1,'DefaultLineLineWidth',3);	% set figure to draw with thick lines by default
grid on							% draw a grid on the figure
hold on							% ensure multiple drawing commands are overlaid on the figure
% without erasing figure first

Field.t = 0;

