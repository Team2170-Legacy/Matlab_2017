%
%   init_Field_v002.m
%
%   Jacob Krucinski
%
%   3/2/2017    Updated with re-arranged field positions: 
%               Starting positions RS1, RS2, RS3, BS1, BS2, BS3
%               Peg positions RP1, RP2, RP3, BP1, BP2, BP3
%
%               Note!
%               Red starting positions RS1 - RS3 are from TOP to BOTTOM
%               Blue starting postions BS1 - BS3 are from BOTTOM to TOP
%               Peg numbers correspond to the starting positions facing the
%               pegs

Field.L         = 55*ft;    % [x-axis] The +1 is for making room for game pieces that are outside of the playing field
Field.W         = 27*ft;    % [y-axis]
Field.t         = 0;

%% Airship Initialization
AirShipRed.x = -488.0333*in + 55*ft;
AirShipRed.y = Field.W / 2;

AirShipRed.sL = 4;      % Airship side lengths [ft]
AirShipRed.R = 40.703*in;

AirShipRed.th(1)      = 30*deg;
AirShipRed.th(2)      = 30*deg + 60*deg;
AirShipRed.th(3)      = 30*deg + 2*60*deg;
AirShipRed.th(4)      = 30*deg + 3*60*deg;
AirShipRed.th(5)      = 30*deg + 4*60*deg;
AirShipRed.th(6)      = 30*deg + 5*60*deg;

for i = 1:6
    AirShipRed.C_x(i)   = AirShipRed.x + AirShipRed.R*cos(AirShipRed.th(i));    % X_coordinate for corner Ci
    AirShipRed.C_y(i)   = AirShipRed.y + AirShipRed.R*sin(AirShipRed.th(i));    % Y_coordinate for corner Ci
end

Field.AirShipRed    = AirShipRed;       % Add red airship to the field
Field.AirShipBlue   = AirShipRed;       % Create blue airship

Field.AirShipBlue.x     = Field.AirShipRed.x + 8;       % Set new x-coordinate for blue airship
Field.AirShipBlue.C_x   = Field.AirShipBlue.C_x + 8;    % Move blue airship __ units to the right


%% Boiler Initialization
BoilerRed.L         = 3*ft + 6*in;
BoilerRed.th        = 135*deg;
BoilerRed.O         = 2*ft + 1*in;

BoilerRed.C1_x      = 0;
BoilerRed.C1_y      = BoilerRed.L * sin(BoilerRed.th);

BoilerRed.C2_x      = BoilerRed.L * -(cos(BoilerRed.th));
BoilerRed.C2_y      = 0;

BoilerRed.M_x       = (BoilerRed.C1_x + BoilerRed.C2_x) / 2;
BoilerRed.M_y       = (BoilerRed.C1_y + BoilerRed.C2_y) / 2;

BoilerRed.ratio     = BoilerRed.O / BoilerRed.L;

% Move from midpoint M towards point C1 by ratio of boiler openeing length to face length 
BoilerRed.C3_x      = BoilerRed.M_x + (BoilerRed.C1_x - BoilerRed.M_x) * BoilerRed.ratio;
BoilerRed.C3_y      = BoilerRed.M_y + (BoilerRed.C1_y - BoilerRed.M_y) * BoilerRed.ratio;

BoilerRed.C4_x      = BoilerRed.M_x + (BoilerRed.C2_x - BoilerRed.M_x) * BoilerRed.ratio;
BoilerRed.C4_y      = BoilerRed.M_y + (BoilerRed.C2_y - BoilerRed.M_y) * BoilerRed.ratio;

Field.BoilerRed = BoilerRed;
Field.BoilerBlue = BoilerRed;

x_reflect = Field.L / 2;     % Reflection line equation for Boilers/Hoppers
                        
Field.BoilerBlue.C1_x = x_reflect * 2;                      % Calculate blue boiler point C1 using x_reflect defined
Field.BoilerBlue.C2_x = x_reflect * 2 - BoilerRed.C2_x;
Field.BoilerBlue.C3_x = x_reflect * 2 - BoilerRed.C3_x;
Field.BoilerBlue.C4_x = x_reflect * 2 - BoilerRed.C4_x;

%% Base Line Initialization

% Red Alliance's baseline
BaseLineRed.C1_x = 9*ft + 4*in;       % Top point of baseline
BaseLineRed.C1_y = 27*ft;       
BaseLineRed.C2_x = 9*ft + 4*in;       % Bottom point of baseline
BaseLineRed.C2_y = 0;

Field.BaseLineRed  = BaseLineRed;
Field.BaseLineBlue = BaseLineRed;

Field.BaseLineBlue.C1_x = 55*ft - BaseLineRed.C1_x;
Field.BaseLineBlue.C2_x = 55*ft - BaseLineRed.C2_x;

%% Elevator Pegs Initialization

% Starting points
RP1.C1_x = (AirShipRed.C_x(2) + AirShipRed.C_x(3)) / 2;
RP1.C1_y = (AirShipRed.C_y(2) + AirShipRed.C_y(3)) / 2;
RP2.C1_x = (AirShipRed.C_x(3) + AirShipRed.C_x(4)) / 2; 
RP2.C1_y = (AirShipRed.C_y(3) + AirShipRed.C_y(4)) / 2;
RP3.C1_x = (AirShipRed.C_x(4) + AirShipRed.C_x(5)) / 2;
RP3.C1_y = (AirShipRed.C_y(4) + AirShipRed.C_y(5)) / 2;

%y_reflect = Field.AirShipRed.y;

% End points
RP1.C2_x = (31.45001 - sqrt(1.74451)) / 8;
RP1.C2_y = -1.732*RP1.C2_x + 11.702;
RP2.C2_x = RP2.C1_x - 13*in; 
RP2.C2_y = RP2.C1_y;
RP3.C2_x = RP1.C2_x;
RP3.C2_y = AirShipRed.y - (RP1.C2_y - AirShipRed.y);

Field.RP1  = RP1;
Field.RP2  = RP2;
Field.RP3  = RP3;

Field.BP1 = RP1;
Field.BP2 = RP2;
Field.BP3 = RP3;

Field.BP1.C1_x = x_reflect * 2 - Field.RP1.C1_x;
Field.BP2.C1_x = x_reflect * 2 - Field.RP2.C1_x;
Field.BP3.C1_x = x_reflect * 2 - Field.RP3.C1_x;
Field.BP1.C2_x = x_reflect * 2 - Field.RP1.C2_x;
Field.BP2.C2_x = x_reflect * 2 - Field.RP2.C2_x;
Field.BP3.C2_x = x_reflect * 2 - Field.RP3.C2_x;

%   Re-arrange pegs: Red pegs were correct, Blue pegs need to be re-ordered
BPTemp     = Field.BP1;
Field.BP1  = Field.BP3;
Field.BP3  = BPTemp;

%% Predefined robot starting positions

% WORK IN PROGRESS

% Starting spot RS3
Field.RS3.x		= Robot.L / 2;      % [m]	Robot center starting position along x-axis
Field.RS3.y		= 1.5;                % [m]	Robot center starting position along y-axis
Field.RS3.theta	= 180*deg;          % [rad]	Robot angle CCW from x-axis

% Starting spot RS2
Field.RS2.x		= Robot.L / 2;      % [m]	Robot center starting position along x-axis
Field.RS2.y		= Field.W / 2;                % [m]	Robot center starting position along y-axis
Field.RS2.theta	= 180*deg;          % [rad]	Robot angle CCW from x-axis

% Starting spot RS1
Field.RS1.x		= Robot.L / 2;      % [m]	Robot center starting position along x-axis
Field.RS1.y		= 7;                % [m]	Robot center starting position along y-axis
Field.RS1.theta	= 180*deg;          % [rad]	Robot angle CCW from x-axis


% Starting spot BS3
Field.BS3.x		= Field.L - Robot.L / 2;        % [m]	Robot center starting position along x-axis
Field.BS3.y   	= Field.RS1.y;                            % [m]	Robot center starting position along y-axis
Field.BS3.theta	= 0*deg;                        % [rad]	Robot angle CCW from x-axis

% Starting spot BS2
Field.BS2.x		= Field.L - Robot.L / 2;        % [m]	Robot center starting position along x-axis
Field.BS2.y		= Field.RS2.y;                            % [m]	Robot center starting position along y-axis
Field.BS2.theta	= 0*deg;                        % [rad]	Robot angle CCW from x-axis

% Starting spot BS1
Field.BS1.x		= Field.L - Robot.L / 2;        % [m]	Robot center starting position along x-axis
Field.BS1.y		= Field.RS3.y;                            % [m]	Robot center starting position along y-axis
Field.BS1.theta	= 0*deg;                        % [rad]	Robot angle CCW from x-axis

%   *** NOT NEEDED, FIXED ABOVE **** Re-arrange starting positions: Red need to be re-ordered, Blue were correct
% 
% RedStartTemp    = Field.RS1;
% Field.RS1       = Field.RS3;
% Field.RS3       = RedStartTemp;


%% Hopper Locations Initialization
% NOTE: THESE DEFINITONS ONLY PLOT THE BASE OF THE HOPPER, NOT THE WHOLE SQUARE
% However we can add these later


% Hopper BASE SET (numbers adjusted for all other hoppers later in code)
for h = 1:5
    Hopper(h).W1 = 2*ft + 2.5*in;
    Hopper(h).W2 = 2*ft + 6*in;
    Hopper(h).W3 = Hopper(h).W1;
end

Hopper(1).x = 13*ft + 9.5*in;
Hopper(1).y = Field.W;

Hopper(2).x = Field.L - (13*ft + 9.5*in) - (Hopper(2).W1 + Hopper(2).W2 + Hopper(2).W3);
Hopper(2).y = Field.W;

Hopper(3).x = 6*ft + 6.5*in;
Hopper(3).y = 0;

Hopper(4).x = x_reflect - (Hopper(4).W1 + Hopper(4).W2 + Hopper(4).W3) / 2;
Hopper(4).y = 0;

Hopper(5).x = Field.L - (6*ft + 6.5*in) - (Hopper(5).W1 + Hopper(5).W2 + Hopper(5).W3);
Hopper(5).y = 0;

Field.Hopper = Hopper;

% Hopper.C_x(1) = 13*ft + 9.5*in;
% Hopper.C_y(2) = Field.W;
% Hopper.C_x(3) = Hopper1.C_x(3) + (2*ft + 2.5*in);
% Hopper.C_y(4) = Field.W;
% 
% Hopper.C_x(3) = Hopper1.C_x(3) + (4*ft + 8.5*in);
% Hopper.C_y(3) = Field.W;
% Hopper.C_x(4) = Hopper1.C_x(4) + (4*ft + 8.5*in);
% Hopper.C_y(4) = Field.W;

% Hopper BASE SET (numbers adjusted for all other hoppers later in code)
% Hopper1.C_x(1) = 13*ft + 9.5*in;
% Hopper1.C_y(1) = Field.W + (1*ft + 11.25*in);
% Hopper1.C_x(2) = Hopper1.C_x(1) + (2*ft + 2.5*in);
% Hopper1.C_y(2) = Hopper1.C_y(1);
% Hopper1.C_x(3) = Hopper1.C_x(1);
% Hopper1.C_y(3) = Field.W;
% Hopper1.C_x(4) = Hopper1.C_x(3) + (2*ft + 2.5*in);
% Hopper1.C_y(4) = Field.W;
% %
% Hopper2.C_x(1) = Hopper1.C_x(1) + (4*ft + 8.5*in);
% Hopper2.C_y(1) = Field.W + (1*ft + 11.25*in);
% Hopper2.C_x(2) = Hopper1.C_x(2) + (4*ft + 8.5*in);
% Hopper2.C_y(2) = Hopper1.C_y(1);
% Hopper2.C_x(3) = Hopper1.C_x(3) + (4*ft + 8.5*in);
% Hopper2.C_y(3) = Field.W;
% Hopper2.C_x(4) = Hopper1.C_x(4) + (4*ft + 8.5*in);
% Hopper2.C_y(4) = Field.W;
% 
% Field.Hopper1 = Hopper1;
% Field.Hopper2 = Hopper2;
% Field.Hopper1 = Hopper1;
% Field.Hopper2 = Hopper1;
% Field.Hopper3 = Hopper1;
% Field.Hopper4 = Hopper1;
% Field.Hopper5 = Hopper1;
% Field.Hopper6 = Hopper1;
% Field.Hopper7 = Hopper1;
% Field.Hopper8 = Hopper1;
% Field.Hopper9 = Hopper1;
% Field.Hopper10 = Hopper1;

% % Hopper 
% for a=1:2:4
%     Field.Hopper2.C_x(a) = 13*ft + 9.5*in;
% end
% 
% for b=2:2:4
%     Field.Hopper2.C_x(b) = (13*ft + 9.5*in) + (4*ft + 8.5*in);
% end
% % Then init y
% 
% for a=1:2:4
%     Field.Hopper3.C_x(a) = Field.L - (13*ft + 9.5*in) - (4*ft + 8.5*in) - (2*ft + 2.5*in);
% end
% 
% for b=2:2:4
%     Field.Hopper3.C_x(b) = Field.L - (13*ft + 9.5*in) - (4*ft + 8.5*in);
% end






