
global deg in ft
%
%   Function to calculate the autonomous robot trajectories
%   for FRC Team 2170 robot
%

%if strcmp(start_pos,'RS3') && strcmp(end_pos,'RB')
    
    % Red.Peg3.x = 0;
    % Red.Peg3.y = 0;
    %
    % Red.Peg2.x = 0;
    % Red.Peg2.y = 0;
    %
    % Red.Peg1.x = 0;
    % Red.Peg1.y = 0;
    
    
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
    x0          = Robot.RS3.x;
    y0          = Robot.RS3.y;
    theta0      = Robot.RS3.theta;
    
    P0_x        = x0;
    P0_y        = y0;
    P0          = [P0_x P0_y];
    theta1_0    = theta0;
    rel_theta1  = -90*deg;      % [rad] robot turn angle in segment 1
    theta1_f	= theta1_0 + rel_theta1;
    
    arcR        = 1.5;          % [m]   Arc radius
    margin      = 0.10%80;         % []    Ratio of max velocity we use in trajectory planning to max velocity
    v_max       = margin * Robot.v_max;
    a_max       = Robot.a_max;
    
    L_1         = arcR*abs(rel_theta1);
    v1			= +v_max;		% drive robot Forward
    a1			= a_max;		% drive robot Forward
    
    t1         = abs(v1)/abs(a1);
    
    
    
%%\    t1          = L_1/abs(v_max) + 1/2*t1a;
    
    
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
    rel_theta2	= -225*deg;
    theta2_f	= theta2_0 + rel_theta2;
    
    %---------------- Segment 3 -------------------------------------------
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
    
    all_x_fwd    = P3_x * v4_0 * (all_t4 - 1/2/t4*all_t4.^2);
    
    
    %----------------	Segment 1 - 4	---------------------------------------
    %   Assemble final trajectory
    
    %   Note! All the all_tx are RELATIVE times.
    %	Now need ABSOLUTE times for the final t_all vector
    all_t       =  [								...
        all_t1										...
        (all_t1(end)+Ts) + all_t2					...
        (all_t1(end)+Ts+all_t2(end)+Ts) + all_t3	];
    
    t3_end          = (all_t1(end)+Ts+all_t2(end)+Ts) + all_t3(end);
    i3_end          = length(all_t1) + length(all_t2) + length(all_t3);
    %
    all_x       = [all_x_t1 all_x_t2 all_x_t3 all_x_t4];
    all_y       = [all_y_t1 all_y_t2 all_y_t3 all_y_t4];
    all_theta   = [all_theta_t1 all_theta_t2 all_theta_t3 ] ;
    
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
    
    %Red Start and End
elseif strcmp(start_pos,'RS3') && strcmp(end_pos,'RP3')
elseif strcmp(start_pos,'RS3') && strcmp(end_pos,'RP2')
elseif strcmp(start_pos,'RS3') && strcmp(end_pos,'RP1')
    
elseif strcmp(start_pos,'RS2') && strcmp(end_pos,'RP3')
elseif strcmp(start_pos,'RS2') && strcmp(end_pos,'RP2')
elseif strcmp(start_pos,'RS2') && strcmp(end_pos,'RP1')
    
elseif strcmp(start_pos,'RS1') && strcmp(end_pos,'RP3')
elseif strcmp(start_pos,'RS1') && strcmp(end_pos,'RP2')
elseif strcmp(start_pos,'RS1') && strcmp(end_pos,'RP1')
    
    
    %Blue Start and End
elseif strcmp(start_pos,'BS3') && strcmp(end_pos,'BP3')
elseif strcmp(start_pos,'BS3') && strcmp(end_pos,'BP2')
elseif strcmp(start_pos,'BS3') && strcmp(end_pos,'BP1')
    
elseif strcmp(start_pos,'BS2') && strcmp(end_pos,'BP3')
elseif strcmp(start_pos,'BS2') && strcmp(end_pos,'BP2')
elseif strcmp(start_pos,'BS2') && strcmp(end_pos,'BP1')
    
elseif strcmp(start_pos,'BS1') && strcmp(end_pos,'BP3')
elseif strcmp(start_pos,'BS1') && strcmp(end_pos,'BP2')
elseif strcmp(start_pos,'BS1') && strcmp(end_pos,'BP1')
end

disp('Done')


return
