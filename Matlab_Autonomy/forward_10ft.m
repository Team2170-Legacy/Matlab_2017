global deg in ft

x0          = Robot.RS3.x;
y0          = Robot.RS3.y;
theta0      = Robot.RS3.theta;


P0_x        = x0;
P0_y        = y0;
P0          = [P0_x P0_y];
theta1_0    = theta0;
rel_theta1  = -90*deg;      % [rad] robot turn angle in segment 1
theta1_f	= theta1_0 + rel_theta1;
margin      = 0.10%80;         % []    Ratio of max velocity we use in trajectory planning to max velocity
v_max       = margin * Robot.v_max;
a_max       = Robot.a_max;

L_1         = 10*ft;
v1			= -v_max;		% drive robot BACKWARDS
a1			= -a_max;		% drive robot BACKWARDS

t1a         = abs(v1)/abs(a1);
t1          = L_1/abs(v_max) + 1/2*t1a;


N1          = round(t1/Ts);
all_t1      = (0:(N1-1))*Ts;

all_theta_t1  = zeros(size(all_t1));


%----------------	Segment 1 + Segment 2  -------------------------------------------
%   1) Acceleration Ramp
for i=1:N1
    t   = all_t1(i);
    if t<=t1a
        xfwd = 0 + 1/2*a1*(t)*(t);
        
    else
        xfwd = 0 + 1/2*a1*(t1a)*(t1a)+v1*(t-t1a);
        
    end
end

t1 = v_max/a1;
t2 = 

%----------------	Segment 3	-------------------------------------------
%   1) DeAcceleration Ramp

    A1 = (t1*v_max)/2;
    A2 = (t2-t1)*v_max;
    A3 = (t3-t2)*v_max/2;
    