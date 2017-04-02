%
%   generate_all_trajectories_v004.m
%
%   Generates all autonomous mode trajectories and writes all .h files
%
%   Martin Krucinski
%   2017-03-19      v001
%
%   Jacob Krucinski
%   2017-03-29      v002    Include the additional three RPx_RB

init_Constants;

%*** NOT NEEDED, init_Robot_v002 now calls init_Field_v002
%**** init_Field_v002;

init_Robot_v002;

%% Initialize table of move time (Auto_Table)
Auto_Table{1}   = {'Start', 'End', 'Time [s]'};

%%
R       = Robot.R;
d       = Robot.d;
L       = Robot.L;

vx0     = 0;
vy0     = 0;
omega0  = 0;
vxf     = 0;
vyf     = 0;
omegaf  = 0;

tfinal      =  2.0;

Ts          = Robot.Ts;

all_start_pos   = {
    'RS1';
    'RS2';
    'RS3';
    
    'BS1';
    'BS2';
    'BS3';
    
    'RP1';
    'RP2';
    'RP3';
    
    'BP1';
    'BP2';
    'BP3';
    
    'RB';
    'BB';
    };

all_end_pos     = {
    'RP1';
    'RP2';
    'RP3';
    
    'BP1';
    'BP2';
    'BP3';
    
    'RB';
    'RB';
    'RB';
    
    'BB';
    'BB';
    'BB';
    
    'F';
    'F';
    };

auto_row = 2;

for traj = 1:length(all_start_pos),
    
    start_pos   = all_start_pos{traj};
    end_pos     = all_end_pos{traj};
    
    disp([ 'Generating trajectory from ' start_pos ' to ' end_pos ]);
    
    [all_omega_R, all_omega_L, all_t,t_auto_end, i_auto_end] = calc_trajectory_v8(start_pos,end_pos,vx0,vy0,omega0,vxf,vyf,omegaf, Robot, Field, Ts);
    
    
    
    if strcmp(start_pos,'RB')
        Robot.StartPos  = Field.BoilerRed;
    elseif strcmp(start_pos,'BB')
        Robot.StartPos  = Field.BoilerBlue;
    else
        Robot.Start_Pos     = eval([ 'Field.' start_pos ]);        % *** NOTE! *** This has to match the starting position for calc_trajectory_v8 ***
    end
    
    t_final     = all_t(end);
    
    make_dot_h_file_v003(start_pos,end_pos,all_omega_R,all_omega_L,all_t,Robot,t_auto_end, i_auto_end);
    
    Auto_Table{auto_row} = { start_pos , end_pos , t_final };
    auto_row = auto_row + 1;
    
    
    
    %%
    
    if 0,
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

Auto_Table = Auto_Table';
cell2table(Auto_Table)

