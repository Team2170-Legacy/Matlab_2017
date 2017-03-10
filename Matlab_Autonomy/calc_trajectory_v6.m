function [all_omega_R, all_omega_L, all_t] = calc_trajectory_v6(x0,y0,theta0,xf,yf,thetaf,vx0,vy0,omega0,vxf,vyf,omegaf, Robot, Ts)
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

%%--------------------------------------------------------------------------
%
%   Trajectory Type 1: From Starting Position to Boiler Low Goal
%
%   Approach is to use 4 segments for the move
%   1) Arc turn
%   2) Straight line, constant velocity
%   3) Arc turn to final robot orientation
%   4) Approach to target with gradual slow-down to 0 velocity

%----------------	Segment 1	-------------------------------------------
%   1) Arc turn, from P0 starting positions to P1 end of arc

P0_x        = x0;
P0_y        = y0;
P0          = [P0_x P0_y];
theta1_0     = theta0;
rel_theta1  = -90*deg;      % [rad] robot turn angle in segment 1
theta1_f	= theta1_0 + rel_theta1;

arcR        = 1.5;          % [m]   Arc radius
margin      = 0.80;         % []    Ratio of max velocity we use in trajectory planning to max velocity
v_max       = margin * Robot.v_max;
a_max       = Robot.a_max;

L_1         = arcR*abs(rel_theta1);
v1			= -v_max;		% drive robot BACKWARDS
a1			= -a_max;		% drive robot BACKWARDS

t1a         = abs(v1)/abs(a1);
t1          = L_1/abs(v_max) + 1/2*t1a;


N1          = round(t1/Ts);
all_t1      = (0:(N1-1))*Ts;

all_theta_t1  = zeros(size(all_t1));

for i=1:N1
    t   = all_t1(i);
    if t<=t1a
        xfwd = 0 + 1/2*a1*(t)*(t);
        
    else
        xfwd = 0 + 1/2*a1*(t1a)*(t1a)+v1*(t-t1a);
        
    end
    theta			= theta1_0 + rel_theta1*abs(xfwd)/L_1;    
    all_theta_t1(i) = theta;
end

C1_x			= x0;
C1_y			= (y0 - arcR);
sweep_angle_t1	= all_theta_t1-90*deg;
all_x_t1		= C1_x + arcR * cos(sweep_angle_t1);
all_y_t1		= C1_y + arcR * sin(sweep_angle_t1);

%   Avoid using integrated end points, calculate end points analytically
%   instead
%	P1_x        = all_x_t1(end);
%	P1_y        = all_y_t1(end);

P1_x        = P0_x + arcR;
P1_y        = P0_y - arcR;
P1          = [  P1_x P1_y ];

%----------------	Segment 2	-------------------------------------------
%   2) Straight down, from P1 to P2
%   Since we don't yet know how far down to move, need to complete
%   segments 3) & 4) first...
%   We do know the P2 x-location however

P2_x        = P1_x;
theta2_0	= theta1_f;
rel_theta2	= 0;
theta2_f	= theta2_0 + rel_theta2;

%----------------	Segment 3	-------------------------------------------
%   3) Arc turn, from P2 to P3 facing target
theta3_0	= theta2_f;
rel_theta3  = -45*deg;      % [rad] robot turn angle towards boiler
theta3_f	= theta3_0 + rel_theta3;

L_3         = arcR*abs(rel_theta3) ;
v3			= -v_max;
t3          = L_3 / abs(v3);
%   calculate the intersection of the Segment 3 arc with the line perpendicular
%   to the low goal
del_x3      = arcR * (1 - cos(rel_theta3));
del_y3      = arcR * sin(abs(rel_theta3));
%   Calculate location of P3
P3_x        = P2_x - del_x3;
P3_y        = P3_x;     % since P3 is on the 45 deg line perpendicular to the low goal
P2_y        = P3_y + del_y3;

P2          = [ P2_x P2_y];
P3          = [ P3_x  P3_y];

%----------------	Segment 2	-------------------------------------------
%   With points P1 & P2 determined, generate trajectory for Segment 2

L_2         = sqrt( (P2_x - P1_x)^2 + (P2_y - P1_y)^2 );    % length of Segment 2
v2			= -v_max;
t2          = L_2 / abs(v2);
N2          = round(t2/Ts);
all_t2      = (0:(N2-1))*Ts;
theta2		= theta2_f;
all_x_t2    = P1_x + v2*cos(theta2)*all_t2;
all_y_t2    = P1_y + v2*sin(theta2)*all_t2;
all_theta_t2= (theta2)* ones(size(all_t2));

%----------------	Segment 3	-------------------------------------------
%   With points P2 & P3 determined, generate trajectory for Segment 3

N3          = round(t3/Ts);
all_t3      = (0:(N3-1))*Ts;
all_theta_t3= theta3_0 + rel_theta3.* all_t3 ./ t3;

%   Calculcate CENTER point C3 for Segment 3 arc
C3_x        = P2_x - arcR;
C3_y        = P2_y;

sweep_angle_t3     = all_theta_t3 - 1/2*pi;
all_x_t3    = C3_x + arcR * cos(sweep_angle_t3);
all_y_t3    = C3_y + arcR * sin(sweep_angle_t3);

%----------------	Segment 4	-------------------------------------------
%   4) Final approach to target, straight line towards target at -135 deg angle, slowing
%   down gradually

%   Set Target destination as temporary Boiler parameters
%   *** Need to provide Field.Boiler to this function ****
Boiler.M_x      = 0.3774;
Boiler.M_y      = 0.3774;

rel_theta4      = 0;
theta4_0		= theta3_f;
theta4_f		= theta4_0 + rel_theta4;

theta4          = theta4_f;
P4_x            = Boiler.M_x + Robot.L/2*cos(theta4);
P4_y            = Boiler.M_x + Robot.L/2*sin(theta4);
P4              = [ P4_x P4_y ];

L_4         = sqrt( (P4_x - P3_x)^2 + (P4_y - P3_y)^2 );    % length of Segment 2
v4_0		= -v_max;
t4          = L_4 / (abs(v4_0)/2);      % average velocity of a slow-down ramp is v_max/2
N4          = round(t4/Ts);
all_t4      = (0:(N4-1))*Ts;

all_x_t4    = P3_x + cos(theta4) * v4_0 * (all_t4 - 1/2/t4*all_t4.^2);
all_y_t4    = P3_x + sin(theta4) * v4_0 * (all_t4 - 1/2/t4*all_t4.^2);
all_theta_t4= (theta4)* ones(size(all_t4));

%----------------	Segment 1 - 4	---------------------------------------
%   Assemble final trajectory

%   Note! All the all_tx are RELATIVE times. 
%	Now need ABSOLUTE times for the final t_all vector
all_t       =  [								...
	 all_t1										...
	(all_t1(end)+Ts) + all_t2					...
	(all_t1(end)+Ts+all_t2(end)+Ts) + all_t3	...
	(all_t1(end)+Ts+all_t2(end)+Ts+all_t3(end)+Ts) + all_t4 ];


%
all_x       = [all_x_t1 all_x_t2 all_x_t3 all_x_t4];
all_y       = [all_y_t1 all_y_t2 all_y_t3 all_y_t4];
all_theta   = [all_theta_t1 all_theta_t2 all_theta_t3 all_theta_t4] ;

all_vx      = [0 diff(all_x)]/Ts;
all_vy      = [0 diff(all_y)]/Ts;
all_omega   = [0 diff(all_theta)]/Ts;

all_v_Fwd_abs   = sqrt(all_vx.^2+all_vy.^2);
all_e_theta		= [ cos(all_theta) ; sin(all_theta) ];
all_v			= [ all_vx	; all_vy ];
all_v_Fwd_sign	= sign( diag(all_e_theta' * all_v )');
all_v_Fwd		= all_v_Fwd_abs .* all_v_Fwd_sign;
all_v_R         = all_v_Fwd + 1/2 * all_omega * Robot.d;
all_v_L         = all_v_Fwd - 1/2 * all_omega * Robot.d;
all_omega_R     = all_v_R / Robot.R;
all_omega_L     = all_v_L / Robot.R;

disp('Done')


return
