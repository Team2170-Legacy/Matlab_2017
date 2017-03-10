
%
%   calc_RS2_RP2_v001
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
%   Trajectory Type : From Starting Position RS2 to Peg RP2
%
%   Approach is to use 2 segments for the move
%
%   1) Straight line, constant velocity
%   2) Approach to target with gradual slow-down to 0 velocity

% Robot starting position selection

Robot.Start_Pos     = Field.RS2;    % CHANGE SPECIFIED STARTING POSITION HERE
                                    % OR
                                    % in the selection of Autonomous
                                    % Trajectory, calc_trajectory_v8.m and
                                    % its associated helper scripts, e.g.
                                    % calc_RS3_RB_v001.m etc.
%   Peg selection
Peg                 = Field.RP2;

Robot.x0        = Robot.Start_Pos.x;
Robot.y0        = Robot.Start_Pos.y;
Robot.theta0    = Robot.Start_Pos.theta;

% Keep radius in case we need to modify move in the future with some slight
% turning
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
%   2) Straight line towards target

theta2_0	= theta1_f;
rel_theta2	= 0*deg;      % [rad] robot turn angle towards peg
theta2_f	= theta2_0 + rel_theta2;


v2_0			= -v_max;
v2_f			= 0;

%   Use same final approach distance as for the other pegs L_3 = 0.9817 m
del_x2          = 0.9817;

P1_y        = P0_y;

P2_x        = Peg.C1_x - Robot.L/2;       % *** Watch sign
P2_y        = P1_y;

P1_x        = P2_x - del_x2;

%   all points P1, P2 x & y have now been calculated

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
%----------------	Segment 2 Completion --------------------------------
%--------------------------------------------------------------------------
%   Final approach to target, straight line towards target at +0 deg angle, 
%   from P1 to P2, slowing down gradually

L_2         = del_x2;    % length of Segment 2
t2          = L_2 / (abs(v2_0)/2);      % average velocity of a slow-down ramp is v_max/2
N2          = round(t2/Ts);
all_t2      = (0:(N2-1))*Ts;

all_x_t2    = P1_x + cos(theta2_f) * v2_0 * (all_t2 - 1/2/t2*all_t2.^2);
all_y_t2    = P1_y + sin(theta2_f) * v2_0 * (all_t2 - 1/2/t2*all_t2.^2);
all_theta_t2= (theta2_f)* ones(size(all_t2));

%--------------------------------------------------------------------------
%----------------	Segment 1 - 2	---------------------------------------
%--------------------------------------------------------------------------
%   Assemble final trajectory
%--------------------------------------------------------------------------

%   Note! All the all_tx are RELATIVE times.
%	Now need ABSOLUTE times for the final t_all vector
all_t       =  [								...
    all_t1										...
    (all_t1(end)+Ts) + all_t2	];

%   Return parameters for end of PURE autonomous mode, 
%   i.e. specify where vision target control blending should start

t_auto_end      = all_t1(end);  
i_auto_end      = length(all_t1);
    
%   Assemble remaining trajectories

all_x       = [all_x_t1 all_x_t2 ];
all_y       = [all_y_t1 all_y_t2 ];
all_theta   = [all_theta_t1 all_theta_t2 ] ;

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
