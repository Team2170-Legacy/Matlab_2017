%	simulate_robot_v008.m
%
%	Jacob Krucinski
%	jacob1576@gmail.com
%
%	FIRST Robotics Team 2170 Titanium Tomahawks
%	Glastonbury High School, Glastonbury, CT, USA
%
%   2017-03-02      Martin Krucinski  Updated with init_Robot_v002,
%   init_Field_v002

%   Initialize conversion constants and field elements
init_Constants;
init_Robot_v002;        % MK init_Robot_v002 now calls init_Field_002

%	initial robot wheel velocities & radius

Robot.wL0		= 0;		% [rad/s]	initial left wheel angular velocity
Robot.wR0		= 0;		% [rad/s]	initial left wheel angular velocity


%	Initialize simulation parameters
Ts			= Robot.Ts;			% [s]		Simulation sample time
tfinal      = all_t(end);

N			= length(all_t);	% []		Number of simulation time points

%	Robot movement simulation

%	Initializations

% Allocate storage for all robot simulation variables
Robot.x_all			= zeros(N,1);	% [m] x-positions for all time-points in simulation
Robot.y_all			= zeros(N,1);	% [m] y-positions
Robot.theta_all		= zeros(N,1);	% [rad] robot angle

Robot.vx_all		= zeros(N,1);	% [m/s]	robot velocities
Robot.vy_all		= zeros(N,1);
Robot.omega_all		= zeros(N,1);	% [rad/s]	robot angular velocities

Robot.wL_all		= zeros(N,1);	% [rad/s]	robot Left wheel angular velocities
Robot.wR_all		= zeros(N,1);	% [rad/s]	robot Right wheel angular velocities
Robot.vFwd_all		= zeros(N,1);	% [m/s]		robot forward velocity (in the direction it is pointing)

v = VideoWriter('Robot_Movie','MPEG-4');	% initialize vide capture of simulation frames
open(v);									% open movie file

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
draw_Field_v002(Field)
                                

%	Initial conditions

Robot.x				= Robot.Start_Pos.x;		% [m] Robot center x-position
Robot.y				= Robot.Start_Pos.y;		% [m] Robot center y-position
Robot.theta			= Robot.Start_Pos.theta;	% [rad] Robot angle CCW from x-axis

Robot.wL			= Robot.wL0;	% [rad/s]	robot Left wheel angular velocity
Robot.wR			= Robot.wR0;	% [rad/s]	robot Right wheel angular velocity

Robot.x_all(1)		= Robot.x;		% Store all robot variables in storage arrays
Robot.y_all(1)		= Robot.y;
Robot.theta_all(1)	= Robot.theta;

Robot.wL_all(1)		= Robot.wL;
Robot.wR_all(1)		= Robot.wR;

Robot.vFwd			= 1/2 * Robot.R * (Robot.wL + Robot.wR);	% Robot Forward velocity, average of the two wheels
Robot.vFwd_all(1)	= Robot.vFwd;

%	Main simulation loop
for i=2:N
	t					= all_t(i);		% [s] get current simulation time
	Robot.t				= t;
    Field.t             = t;
	
	Robot.wL			= all_omega_L(i);			% [rad/s]	Trajectory planned wheel velocities
	Robot.wR			= all_omega_R(i);
	
	Robot.vFwd			= 1/2 * Robot.R * (Robot.wL + Robot.wR);		% [m/s] Robot Forward velocity, average of the two wheels
	Robot.omega			= (Robot.wR - Robot.wL) * Robot.R /  Robot.d;	% Robot Angular velocity
	
	Robot.vx			= Robot.vFwd * cos( Robot.theta );		% [m/s]		Robot center x-velocity
	Robot.vy			= Robot.vFwd * sin( Robot.theta );		% [m/s]		Robot center y-velocity
	
	Robot.x				= Robot.x + Robot.vx * Ts;			% [m]	Integrate robot x-position
	Robot.y				= Robot.y + Robot.vy * Ts;			% [m]	Integrate robot y-position
	Robot.theta			= Robot.theta + Robot.omega * Ts;	% [rad]	Integrate robot angle
	
	draw_Robot(Robot);						% Call function to draw Robot in figure
    draw_Field_v002(Field);
	Robot_Figure		= getframe(f1);		% Capture screenshot image of figure
	Robot_Image			= Robot_Figure.cdata;
%	pause

	if i < N
        cla         % Erase figure in preparation for next simulation step
    end
	
	Robot.x_all(i)		= Robot.x;		% Store all robot variables in storage arrays
	Robot.y_all(i)		= Robot.y;
	Robot.theta_all(i)	= Robot.theta;
	
	Robot.wL_all(i)		= Robot.wL;
	Robot.wR_all(i)		= Robot.wR;
	
	writeVideo(v, Robot_Image);			% Write screenshot image to video file
end

close(v)			% Close robot simulation video file

f2		= figure;				% open figure
set(f2,'DefaultLineLineWidth',3);	% set figure to draw with thick lines by default
grid on							% draw a grid on the figure
hold on
plot(all_t, Robot.wL_all, 'b');	% Plot left wheel velocities in Blue
plot(all_t, Robot.wR_all, 'r');	% Plot right wheel velocities in Red
hold off
xlabel('t [s]')
ylabel('omega [rad/s]')


f3		= figure;				% open figure
set(f3,'DefaultLineLineWidth',3);	% set figure to draw with thick lines by default

subplot(311)
title('Robot positions')
plot(all_t, Robot.x_all);	% Plot robot x-positions
grid on							% draw a grid on the figure
ylabel('x [m]')

subplot(312)
plot(all_t, Robot.y_all);	% Plot robot y-positions
grid on							% draw a grid on the figure
ylabel('y [m]')

subplot(313)
plot(all_t, Robot.theta_all);	% Plot robot angles
grid on							% draw a grid on the figure
ylabel('theta [rad]')

xlabel('t [s]')


