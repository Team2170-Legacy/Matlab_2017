%
%   forward_10ft_v002
%
global deg in ft

x0          = Robot.RS3.x;
y0          = Robot.RS3.y;
theta0      = Robot.RS3.theta;


P0_x        = x0;
P0_y        = y0;
P0          = [P0_x P0_y];
theta1_0    = theta0;
rel_theta1  = 0*deg;      % [rad] robot turn angle in segment 1
theta1_f	= theta1_0 + rel_theta1;
margin      = 0.30%80;         % []    Ratio of max velocity we use in trajectory planning to max velocity
v_max       = margin * Robot.v_max;
a_max       = margin * Robot.a_max;

L_path      = 10*ft;
v1			= -v_max;		% drive robot BACKWARDS
a1			= -a_max;		% drive robot BACKWARDS

%   Note!
%   trx - relative time
%   tx  - absolute time

tr1         = abs(v1)/abs(a1);
N1          = round(tr1/Ts);
all_t1      = (0:(N1-1))*Ts;

all_theta_t1    = zeros(size(all_t1));
all_v_t1        = zeros(size(all_t1));

%----------------	Segment 1  -------------------------------------------
%   1) Acceleration Ramp

for i=1:N1
    t   = all_t1(i);
    all_v_t1(i) = t/tr1 * v1;
    all_x_t1
end


%----------------	Segment 2	-------------------------------------------
%   2) Constant velocity

L_seg1      = ( 1/2*v1*tr1 );
L_seg3      = L_seg1;
L_seg2     = L_path - L_seg1 - L_seg3; 

v2          = v1;
tr2         = L_seg2 / abs(v2);

N2          = round(tr2/Ts);
all_tr2      = (0:(N2-1))*Ts;



%----------------	Segment 3	-------------------------------------------
%   3) DeAcceleration Ramp

    