%% Simulation sampling time
Robot.Ts        = 0.010;        % [s]

%% Robot dimensions and velocity
Robot.d	= 21*in;		% distance between wheel centerlines
Robot.R = 2*in;         % [m]		Robot wheel radius
Robot.L = 22*in;		% Robot length (along the driving direction)
Robot.W = 24.5*in;
Robot.v_max = 3.048;    % Robot max velocity (on each R / L drive wheel)
Robot.a_max = 3;        %Robot assumed acceleration
%% Predefined robot starting positions

% WORK IN PROGRESS

% Starting spot RS3
Robot.RS3.x		= Robot.L / 2;      % [m]	Robot center starting position along x-axis
Robot.RS3.y		= 7;                % [m]	Robot center starting position along y-axis
Robot.RS3.theta	= 180*deg;          % [rad]	Robot angle CCW from x-axis

% Starting spot RS2
Robot.RS2.x		= Robot.L / 2;      % [m]	Robot center starting position along x-axis
Robot.RS2.y		= 4;                % [m]	Robot center starting position along y-axis
Robot.RS2.theta	= 180*deg;          % [rad]	Robot angle CCW from x-axis

% Starting spot RS1
Robot.RS1.x		= Robot.L / 2;      % [m]	Robot center starting position along x-axis
Robot.RS1.y		= 3;                % [m]	Robot center starting position along y-axis
Robot.RS1.theta	= 180*deg;          % [rad]	Robot angle CCW from x-axis


% Starting spot BS3
Robot.BS3.x		= Field.L - Robot.L / 2;        % [m]	Robot center starting position along x-axis
Robot.BS3.y   	= 7;                            % [m]	Robot center starting position along y-axis
Robot.BS3.theta	= 0*deg;                        % [rad]	Robot angle CCW from x-axis

% Starting spot BS2
Robot.BS2.x		= Field.L - Robot.L / 2;        % [m]	Robot center starting position along x-axis
Robot.BS2.y		= 4;                            % [m]	Robot center starting position along y-axis
Robot.BS2.theta	= 0*deg;                        % [rad]	Robot angle CCW from x-axis

% Starting spot BS1
Robot.BS1.x		= Field.L - Robot.L / 2;        % [m]	Robot center starting position along x-axis
Robot.BS1.y		= 3;                            % [m]	Robot center starting position along y-axis
Robot.BS1.theta	= 0*deg;                        % [rad]	Robot angle CCW from x-axis

% Robot starting position selection

Robot.Start_Pos     = Robot.RS3;    % CHANGE SPECIFIED STARTING POSITION HERE
                                    % OR
                                    % in the selection of Autonomous
                                    % Trajectory, calc_trajectory_v8.m and
                                    % its associated helper scripts, e.g.
                                    % calc_RS3_RB_v001.m etc.

Robot.x0 = Robot.Start_Pos.x;
Robot.y0 = Robot.Start_Pos.y;
Robot.theta0 = Robot.Start_Pos.theta;
