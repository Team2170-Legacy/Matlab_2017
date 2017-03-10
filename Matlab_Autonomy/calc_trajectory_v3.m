function [all_omega_R, all_omega_L, all_t] = calc_trajectory_v3(x0,y0,theta0,xf,yf,thetaf,vx0,vy0,omega0,vxf,vyf,omegaf, Robot, Ts)
global deg in ft
%
%   Function to calculate the autonomous robot trajectories 
%   for FRC Team 2170 robot
%
%   x0      - [m]   initial robot center x position
%   y0      - [m]   initial robot center x position
%   theta0  - [rad] initial robot angle (between FWD direction and x-axis)

%   xf      - [m]   final robot center x position
%   yf      - [m]   final robot center x position
%   thetaf  - [rad] final robot angle (between FWD direction and x-axis)

%   vx0      - [m/s]   initial robot center x velocity
%   vy0      - [m/s]   initial robot center x velocity
%   omega0   - [rad/s] initial robot angular velocity (between FWD direction and x-axis)

%   vxf      - [m/s]   final robot center x velocity
%   vyf      - [m/s]   final robot center x velocity
%   omegaf   - [rad/s] final robot angular velocity (between FWD direction and x-axis)

%  
%   Approach is to use 4 segments for the move:
%   1) Arc turn
%   2) Straight line, constant velocity
%   3) Arc turn to final robot orientation
%   4) Approach to target with gradual slow-down to 0 velocity

%   1) Arc turn, from P0 starting positions to P1 end of arc


arcR        = 1.5;          % [m]   Arc radius
theta1      = -90*deg;      % [rad] robot angle at end of arc
margin      = 0.80;         % []    Ratio of max velocity we use in trajectory planning to max velocity
v_max       = margin * Robot.v_max;

t1          = arcR*abs(theta1) / v_max;
N1          = round(t1/Ts);
all_t1      = (0:(N1-1))*Ts;
all_theta_t1= theta0 + theta1 .* all_t1 ./ t1;

all_x_t1    = x0 + arcR * cos(all_theta_t1);
all_y_t1    = x0 + arcR * sin(all_theta_t1);

%   2) Straight down, from P1 to P2
%   TO BE DONE!

%   3) Arc turn, from P2 to P3 facing target
%   TO BE DONE!

%   4) Finial approach to target, straight line towards target at -135 deg angle, slowing
%   down gradually
%   TO BE DONE!

all_t       = all_t1;
all_x       = all_x_t1;
all_y       = all_y_t1;
all_theta   = all_theta_t1;

all_vx      = [0 diff(all_x)]/Ts;
all_vy      = [0 diff(all_y)]/Ts;
all_omega   = [0 diff(all_theta)]/Ts;

all_v_Fwd       = sqrt(all_vx.^2+all_vy.^2);

all_v_R         = all_v_Fwd + 1/2 * all_omega * Robot.d;
all_v_L         = all_v_Fwd - 1/2 * all_omega * Robot.d;
all_omega_R     = all_v_R / Robot.R;
all_omega_L     = all_v_L / Robot.R;

disp('Done')


return
