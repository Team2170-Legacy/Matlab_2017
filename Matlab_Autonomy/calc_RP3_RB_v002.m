
%
%   calc_RP3_RB_v002.m
%
%   03/29/2017
%   Jacob Krucinski (jacob1576@gmail.com)


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
%   Trajectory Type 1: From Peg to Boiler Low Goal
%
%   Approach is to use 4 segments for the move
%   1) Arc turn
%   2) Straight line, constant velocity
%   3) Arc turn to final robot orientation
%   4) Approach to target with gradual slow-down to 0 velocity

% Robot starting position selection

Robot.Start_Pos     = Field.RP3;    % CHANGE SPECIFIED STARTING POSITION HERE
                                    % OR
                                    % in the selection of Autonomous
                                    % Trajectory, calc_trajectory_v8.m and
                                    % its associated helper scripts, e.g.
                                    % calc_RS3_RB_v001.m etc.

Robot.theta0    = Robot.Start_Pos.th;
Robot.x0        = Robot.Start_Pos.C1_x + Robot.L/2*cos(Robot.theta0);
Robot.y0        = Robot.Start_Pos.C1_y + Robot.L/2*sin(Robot.theta0);


%----------------	Segment 1	-------------------------------------------
%   1) Arc turn, from P0 starting positions to P1 end of arc
x0          = Robot.x0;
y0          = Robot.y0;
theta0      = Robot.theta0;

P0_x        = x0;
P0_y        = y0;
P0          = [P0_x P0_y];
theta1_0    = theta0;
rel_theta1  = -17*deg;      % [rad] robot turn angle in segment 1
theta1_f	= theta1_0 + rel_theta1;

arcR        = 1.5;          % [m]   Arc radius
margin      = 0.80;         % []    Ratio of max velocity we use in trajectory planning to max velocity
v_max       = margin * Robot.v_max;
a_max       = Robot.a_max;

L_1         = arcR*abs(rel_theta1);
v1			= +v_max;		% drive robot FORWARDS
a1			= +a_max;		% drive robot FORWARDS

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


C1_x			= x0 + arcR * cos(150*deg);
C1_y			= y0 + arcR * sin(150*deg);
sweep_angle_t1	= all_theta_t1 + 90*deg;
all_x_t1		= C1_x + arcR * cos(sweep_angle_t1);
all_y_t1		= C1_y + arcR * sin(sweep_angle_t1);

%   Avoid using integrated end points, calculate end points analytically
%   instead

%*** MKrucinski 03/29/17  
% For RP1_RB, actually DO use the end points, easier to calculate

    P1_x        = all_x_t1(end);
	P1_y        = all_y_t1(end);

%P1_x        = P0_x - arcR;
%P1_y        = P0_y - arcR;

P1          = [ P1_x P1_y ];

theta2_0	= theta1_f;
rel_theta2	= 0;
theta2_f	= theta2_0 + rel_theta2;


%----------------	Segment 4	-------------------------------------------
%   4) Final approach to target, straight line towards target at -135 deg angle, slowing
%   down gradually

%   Set Target destination as temporary Boiler parameters
%   *** Need to provide Field.Boiler to this function ****
% DONE! See below ****  Boiler.M_x      = 0.3774;
% DONE! See below ****  Boiler.M_y      = 0.3774;

Boiler = Field.BoilerRed;


rel_theta4      = 0;
theta4_0		= theta2_f;
theta4_f		= theta4_0 + rel_theta4;

theta4          = theta4_f;
P4_x            = Boiler.M_x - Robot.L/2*cos(theta4);
P4_y            = Boiler.M_y - Robot.L/2*sin(theta4);
P4              = [ P4_x P4_y ];

L_4         = Robot.L;    % length of Segment 4

%----------------	Segment 2	-------------------------------------------
%   2) Straight towards Boiler

P2_x        = P4_x - L_4*cos(theta4);
P2_y        = P4_y - L_4*sin(theta4);

theta2_0	= theta1_f;
rel_theta2	= 0;
theta2_f	= theta2_0 + rel_theta2;

P2          = [ P2_x P2_y];
%----------------	Segment 2	-------------------------------------------
%   With points P1 & P2 determined, generate trajectory for Segment 2

L_2         = sqrt( (P2_x - P1_x)^2 + (P2_y - P1_y)^2 );    % length of Segment 2
v2			= +v_max;
t2          = L_2 / abs(v2);
N2          = round(t2/Ts);
all_t2      = (0:(N2-1))*Ts;
theta2		= theta2_f;
all_x_t2    = P1_x + v2*cos(theta2)*all_t2;
all_y_t2    = P1_y + v2*sin(theta2)*all_t2;
all_theta_t2= (theta2)* ones(size(all_t2));
 



%----------------	Segment 4, Part 2 -------------------------------------------

v4_0		= +v_max;
t4          = L_4 / (abs(v4_0)/2);      % average velocity of a slow-down ramp is v_max/2
N4          = round(t4/Ts);
all_t4      = (0:(N4-1))*Ts;

all_x_t4    = P2_x + cos(theta4) * v4_0 * (all_t4 - 1/2/t4*all_t4.^2);
all_y_t4    = P2_y + sin(theta4) * v4_0 * (all_t4 - 1/2/t4*all_t4.^2);
all_theta_t4= (theta4)* ones(size(all_t4));

%----------------	Segment 1 - 4	---------------------------------------
%   Assemble final trajectory

%   Note! All the all_tx are RELATIVE times.
%	Now need ABSOLUTE times for the final t_all vector
all_t       =  [								...
    all_t1										...
    (all_t1(end)+Ts) + all_t2					...
    (all_t1(end)+Ts+all_t2(end)+Ts) + all_t4 ];

t3_end          = 0;
i3_end          = 0;
%
all_x       = [all_x_t1 all_x_t2  all_x_t4];
all_y       = [all_y_t1 all_y_t2 all_y_t4];
all_theta   = [all_theta_t1 all_theta_t2 all_theta_t4] ;

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

t_auto_end      = all_t(end);  
i_auto_end      = length(all_t);

