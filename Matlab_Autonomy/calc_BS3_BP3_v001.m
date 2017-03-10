
%
%   calc_BS3_BP3_v001.m
%
%   03/02/2017


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
%   Trajectory Type : From Starting Position BS3 to Peg BP3
%
%   Approach is to use 3 segments for the move
%
%   1) Straight line, constant velocity
%   2) Arc turn to final robot orientation
%   3) Approach to target with gradual slow-down to 0 velocity

% Robot starting position selection

Robot.Start_Pos     = Field.BS3;    % CHANGE SPECIFIED STARTING POSITION HERE
                                    % OR
                                    % in the selection of Autonomous
                                    % Trajectory, calc_trajectory_v8.m and
                                    % its associated helper scripts, e.g.
                                    % calc_RS3_RB_v001.m etc.
%   Peg selection
Peg                 = Field.BP3;

Robot.x0        = Robot.Start_Pos.x;
Robot.y0        = Robot.Start_Pos.y;
Robot.theta0    = Robot.Start_Pos.theta;

arcR        = 1.5;          % [m]   Arc radius
margin      = 0.80;         % []    Ratio of max velocity we use in trajectory planning to max velocity
v_max       = margin * Robot.v_max;
a_max       = Robot.a_max;

%--------------------------------------------------------------------------
%----------------	Segment 1	-------------------------------------------
%--------------------------------------------------------------------------
%   1) Straight line, acceleration ramp followed by constant velocity
%   from P0 to P1
%   Since we don't yet know how far to the right to move before turning, need to complete
%   segments 2) & 3) first...
%   We do know the P1 y-location however

x0          = Robot.x0;
y0          = Robot.y0;
theta0      = Robot.theta0;

P0_x        = x0;
P0_y        = y0;
P0          = [P0_x P0_y];
theta1_0    = theta0;
rel_theta1  = -0*deg;      % [rad] robot turn angle in segment 1
theta1_f	= theta1_0 + rel_theta1;

v1			= -v_max;		% drive robot BACKWARDS
a1			= -a_max;		% drive robot BACKWARDS

%--------------------------------------------------------------------------
%----------------	Segment 2	-------------------------------------------
%--------------------------------------------------------------------------
%   2) Arc turn to final robot orientation

theta2_0	= theta1_f;
rel_theta2	= +60*deg;      % [rad] robot turn angle towards peg
theta2_f	= theta2_0 + rel_theta2;
L_2         = arcR*abs(rel_theta2);

v2			= -v_max;
t2          = L_2 / abs(v2);

%   calculate the size of the arc turn

del_x2      = arcR * sin(abs(rel_theta2));
del_y2      = arcR * (1 - cos(abs(rel_theta2)));

%--------------------------------------------------------------------------
%----------------	Segment 3	-------------------------------------------
%--------------------------------------------------------------------------
%   3) Straight line towards target

theta3_0	= theta2_f;
rel_theta3	= 0*deg;      % [rad] robot turn angle towards peg
theta3_f	= theta3_0 + rel_theta3;


v3_0			= -v_max;
v3_f			= 0;


%   calculate the intersection of the Segment 2 arc with the line perpendicular
%   to the peg, and the associated points P1 & P2

P1_y        = P0_y;
P2_y        = P0_y - del_y2;

%   Now, given the y-coord of P2, we can calculate P2_x,
%   since it is on the line perpendicular to the gear target,
%   at an angle of theta3_f = 120 deg (90 + 30) 

P3_x        = Peg.C1_x + Robot.L/2*cos(theta3_f);
P3_y        = Peg.C1_y + Robot.L/2*sin(theta3_f);

del_y3      = P3_y - P2_y;
del_x3      = del_y3 / sin(theta3_f) * cos(theta3_f);

P2_x        = P3_x - del_x3;

P1_x        = P2_x + del_x2;        % NOTE! Sign change compared to RED side

%   all points P1, P2, P3 x & y have now been calculated

%--------------------------------------------------------------------------
%----------------	Segment 1	Completion --------------------------------
%--------------------------------------------------------------------------

t1a         = abs(v1)/abs(a1);

L_1         = abs(P1_x - P0_x);
t1          = L_1/abs(v_max) + 1/2*t1a;


N1          = round(t1/Ts);
all_t1      = (0:(N1-1))*Ts;

all_x_t1  = zeros(size(all_t1));

for i=1:N1
    t   = all_t1(i);
    if t<=t1a
        xfwd = 0 + 1/2*a1*(t)*(t);
        
    else
        xfwd = 0 + 1/2*a1*(t1a)*(t1a)+v1*(t-t1a);
        
    end
    x1			= P0_x + xfwd * cos(theta1_0);
    all_x_t1(i) = x1;
end


all_theta_t1    = (theta1_f)* ones(size(all_t1));
all_y_t1        = P0_y * ones(size(all_t1));

%--------------------------------------------------------------------------
%----------------	Segment 2	Completion --------------------------------
%--------------------------------------------------------------------------

C2_x			= P1_x;
C2_y			= (P1_y - arcR);

t2          = L_2/abs(v_max);

N2          = round(t2/Ts);
all_t2      = (0:(N2-1))*Ts;

all_theta_t2    = theta2_0 + rel_theta2.* all_t2 ./ t2;
sweep_angle_t2	= all_theta_t2+90*deg;        % NOTE! Sign change compared to RED side
all_x_t2		= C2_x + arcR * cos(sweep_angle_t2);
all_y_t2		= C2_y + arcR * sin(sweep_angle_t2);

%--------------------------------------------------------------------------
%----------------	Segment 3	Completion --------------------------------
%--------------------------------------------------------------------------
%   Final approach to target, straight line towards target at +120 deg angle, 
%   from P2 to P3, slowing down gradually

L_3         = sqrt( (P3_x - P2_x)^2 + (P3_y - P2_y)^2 );    % length of Segment 2
t3          = L_3 / (abs(v3_0)/2);      % average velocity of a slow-down ramp is v_max/2
N3          = round(t3/Ts);
all_t3      = (0:(N3-1))*Ts;

all_x_t3    = P2_x + cos(theta3_f) * v3_0 * (all_t3 - 1/2/t3*all_t3.^2);
all_y_t3    = P2_y + sin(theta3_f) * v3_0 * (all_t3 - 1/2/t3*all_t3.^2);
all_theta_t3= (theta3_f)* ones(size(all_t3));

%--------------------------------------------------------------------------
%----------------	Segment 1 - 3	---------------------------------------
%--------------------------------------------------------------------------
%   Assemble final trajectory
%--------------------------------------------------------------------------

%   Note! All the all_tx are RELATIVE times.
%	Now need ABSOLUTE times for the final t_all vector
all_t       =  [								...
    all_t1										...
    (all_t1(end)+Ts) + all_t2					...
    (all_t1(end)+Ts+all_t2(end)+Ts) + all_t3];

t2_end          = all_t1(end)+Ts+all_t2(end);
i2_end          = length(all_t1) + length(all_t2);

%   Return parameters for end of PURE autonomous mode, 
%   i.e. specify where vision target control blending should start

t_auto_end      = all_t1(end) + Ts + all_t2(end);  
i_auto_end      = length(all_t1) + length(all_t2);
    
%   Assemble remaining trajectories

all_x       = [all_x_t1 all_x_t2 all_x_t3];
all_y       = [all_y_t1 all_y_t2 all_y_t3];
all_theta   = [all_theta_t1 all_theta_t2 all_theta_t3] ;

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
