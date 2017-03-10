%	simulate_robot_v002.m
%
%	Jacob Krucinski
%	jacob1576@gmail.com
%
%	FIRST Robotics Team 2170 Titanium Tomahawks
%	Glastonbury High School, Glastonbury, CT, USA

%   Initialize conversion constants and field elements
init_Constants;
init_Field;

%	Initialize robot dimensions
Robot.L = 20*in;		% Robot length (along the driving direction)
Robot.W = 30*in;		% Robot width (perpendicular to driving direction)
Robot.d	= 25*in;		% distance between wheel centerlines

%	initial position & angle
Robot.x0		= 1;	% [m]	Robot center starting position along x-axis
Robot.y0		= 2;	% [m]	Robot center starting position along y-axis
Robot.theta0	= 30*deg;	% [rad]	Robot angle CCW from x-axis

%	initial robot wheel velocities & radius
Robot.R			= 5/2*in;	% [m]		Robot wheel radius
Robot.wL0		= 2;		% [rad/s]	initial left wheel angular velocity
Robot.wR0		= 1;		% [rad/s]	initial left wheel angular velocity

%   Initalize field

init_Field

%	Initialize simulation parameters
Ts			= 0.100;			% [s]		Simulation sample time
tfinal		= 30;				% [s]		Simulation end time
t_all		= 0:Ts:tfinal;		% [s]		All Simulation time points array
N			= length(t_all);	% []		Number of simulation time points

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
xlim([-1*ft Field.L])					% [m]	set figure limits for x-axis
ylim([-2*ft Field.W])					% [m]	set figure limits for y-axis
set(f1,'DefaultLineLineWidth',3);	% set figure to draw with thick lines by default
grid on							% draw a grid on the figure
hold on							% ensure multiple drawing commands are overlaid on the figure
                                % without erasing figure first

Field.t = 0;                                
draw_Field(Field)
                                

%	Initial conditions

Robot.x				= Robot.x0;		% [m] Robot center x-position
Robot.y				= Robot.y0;		% [m] Robot center y-position
Robot.theta			= Robot.theta0;	% [rad] Robot angle CCW from x-axis

Robot.wL			= Robot.wL0;	% [rad/s]	robot Left wheel angular velocity
Robot.wR			= Robot.wR0;	% [rad/s]	robot Right wheel angular velocity

Robot.x_all(1)		= Robot.x;		% Store all robot variables in storage arrays
Robot.y_all(1)		= Robot.y;
Robot.theta_all(1)	= Robot.theta;

Robot.wL_all(1)		= Robot.wL;
Robot.wR_all(1)		= Robot.wR;

Robot.vFwd			= 1/2 * Robot.R * (Robot.wL + Robot.wR);	% Robot Forward velocity, average of the two wheels
Robot.vFwd_all(1)	= Robot.vFwd;

Period			= tfinal / 4;		% period & frequency for example wheel velcoity sine-waves
freq			= 1/Period;

%	Main simulation loop
for i=2:N
	t					= t_all(i);		% [s] get current simulation time
	Robot.t				= t;
    Field.t             = t;
	
	Robot.wL			= Robot.wL0 + 1.5 * sin(2*pi*freq*t);			% [rad/s]	Example sinusoidal wheel velocities
	Robot.wR			= Robot.wR0 + 0.75 * (-sin(2*pi*freq*t));
	
	Robot.vFwd			= 1/2 * Robot.R * (Robot.wL + Robot.wR);		% [m/s] Robot Forward velocity, average of the two wheels
	Robot.omega			= (Robot.wR - Robot.wL) * Robot.R /  Robot.d;	% Robot Angular velocity
	
	Robot.vx			= Robot.vFwd * cos( Robot.theta );		% [m/s]		Robot center x-velocity
	Robot.vy			= Robot.vFwd * sin( Robot.theta );		% [m/s]		Robot center y-velocity
	
	Robot.x				= Robot.x + Robot.vx * Ts;			% [m]	Integrate robot x-position
	Robot.y				= Robot.y + Robot.vy * Ts;			% [m]	Integrate robot y-position
	Robot.theta			= Robot.theta + Robot.omega * Ts;	% [rad]	Integrate robot angle
	
	draw_Robot(Robot);						% Call function to draw Robot in figure
    draw_Field(Field);
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
	
	%writeVideo(v, Robot_Image);			% Write screenshot image to video file
end


close(v)			% Close robot simulation video file

f2		= figure;				% open figure
set(f2,'DefaultLineLineWidth',3);	% set figure to draw with thick lines by default
grid on							% draw a grid on the figure
hold on
plot(t_all, Robot.wL_all, 'b');	% Plot left wheel velocities in Blue
plot(t_all, Robot.wR_all, 'r');	% Plot right wheel velocities in Red
hold off

f3		= figure;				% open figure
set(f3,'DefaultLineLineWidth',3);	% set figure to draw with thick lines by default

subplot(311)
title('Robot positions')
plot(t_all, Robot.x_all);	% Plot robot x-positions
grid on							% draw a grid on the figure
ylabel('x [m]')

subplot(312)
plot(t_all, Robot.y_all);	% Plot robot y-positions
grid on							% draw a grid on the figure
ylabel('y [m]')

subplot(313)
plot(t_all, Robot.theta_all);	% Plot robot angles
grid on							% draw a grid on the figure
ylabel('theta [rad]')

xlabel('t [s]')


