%
%   make_h_files_v008.m
%
%   Generate all .h files for all trajectories
%
%   Martin Krucinski

init_Constants;

%*** NOT NEEDED, init_Robot_v002 now calls init_Field_v002
%**** init_Field_v002;

init_Robot_v002;

R       = Robot.R;
d       = Robot.d;
L       = Robot.L;


% x0 = Robot.x0;
% y0 = Robot.y0;
% theta0 = Robot.theta0;

% xf      = 1;
% yf      = 1;
% thetaf  = -135*deg;


vx0     = 0;
vy0     = 0;
omega0  = 0;
vxf     = 0;
vyf     = 0;
omegaf  = 0;

tfinal      =  2.0;

Ts          = Robot.Ts;

all_start_pos{1}   = 'RS1';    all_end_pos{1}     = 'RP1';
all_start_pos{2}   = 'BS3';    all_end_pos{2}     = 'BP3';
all_start_pos{3}   = 'RS3';    all_end_pos{3}     = 'RP3';
all_start_pos{4}   = 'BS1';    all_end_pos{4}     = 'BP1';
all_start_pos{5}   = 'RS2';    all_end_pos{5}     = 'RP2';
all_start_pos{6}   = 'BS2';    all_end_pos{6}     = 'BP2';

num_traj = length(all_start_pos);

for t = 1:num_traj,
    
    start_pos   = all_start_pos{t};
    end_pos     = all_end_pos{t};
    
    
    [all_omega_R, all_omega_L, all_t,t_auto_end, i_auto_end] = calc_trajectory_v8(start_pos,end_pos,vx0,vy0,omega0,vxf,vyf,omegaf, Robot, Field, Ts);
    %   Calculate trajectories
    
    Robot.Start_Pos     = eval([ 'Field.' start_pos ]);        % *** NOTE! *** This has to match the starting position for calc_trajectory_v8 ***
    
    t_final     = all_t(end);
    
    disp([ 'Writing .h file for trajectory:   From: ' start_pos '       To:     ' end_pos ]);

    make_dot_h_file_v002(start_pos,end_pos,all_omega_R,all_omega_L,all_t,Robot,t_auto_end, i_auto_end);
    
    
    %%
    
    do_plots = 0;
    
    if do_plots,
        f3=figure;
        title('Wheel Angular Velocities')
        set(gcf, 'DefaultLineLineWidth', 3);
        
        subplot(211)
        plot(all_t, all_omega_R,'r')
        ylabel('Right Wheel Velocity [rad/s]')
        grid on
        
        subplot(212)
        plot(all_t, all_omega_L,'b')
        ylabel('Left Wheel Velocity [rad/s]')
        xlabel('t [s]')
        grid on
    end
    
end
