
%
%   calc_BB_F_v001.m
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


calc_RB_F_v001

%   Mirror the RED trajectory (RB_F in this case)


Robot.Start_Pos     = Field.BoilerBlue;    % CHANGE SPECIFIED STARTING POSITION HERE
% OR
% in the selection of Autonomous
% Trajectory, calc_trajectory_v8.m and
% its associated helper scripts, e.g.
% calc_RS3_RB_v001.m etc.

Robot.theta0    = -45*deg;
Robot.x0        = Field.BoilerBlue.M_x - Robot.L/2*cos(Robot.theta0);
Robot.y0        = Field.BoilerBlue.M_y - Robot.L/2*sin(Robot.theta0);

all_x       = Field.L - all_x;
all_y       = all_y;
all_theta   = 180*deg - all_theta ;

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

