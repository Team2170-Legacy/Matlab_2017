function [Ctheta, Cx, Cy] = calc_trajectory(x0,y0,theta0,xf,yf,thetaf,vx0,vy0,omega0,vxf,vyf,omegaf,tfinal, d)
%
%   Function to calculate the autonomous robot trajectories 
%   for FRC Team 2170 robot
%
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

%   WRONG!!!
%   Approach is to assume that each Left and Right wheel velocity is a
%   5th order polynomial (6 unknown coefficients) for the 12 constraint
%   parameters

%   CORRECT ????
%   Approach is to assume that:
%   x is a 4th order polynomial     (5 unknown coeff)
%   theta is a 5th order polynomial (6 unknown coefficients) 
%   Integrating & solving for y should yield a 12th unknown coefficient
%   Should be solvable since we have 12 constraint equations above 
%   (without acceleration constraints for now)


%   Solve coefficients for initial time t=0 (start of move)

%   Utilize equations:
%   v_Fwd   = (vR + vL)/2
%   omega   = (vR - vL)/d       (d is distance between wheel centerlines)

%   Set up equation system with V = M * C
%
%   where C = [ Cth0 Cth1 Cth2 Cth3 Cx0 Cx1 Cx2 Cx3]'
%
%   Start with 3rd order polynomail for x
%   and 3rd order polynomail for theta

%%  for x & theta
V          = [ x0 ; xf ; vx0 ; vxf ; theta0 ; thetaf ; omega0 ; omegaf ];


t           = tfinal;
t2          = t^2;
t3          = t^3;
t4          = t^4;
t5          = t^5;

M          = [ ...
    0       0       0       0       1       0       0       0       ; ...       % for x0
    0       0       0       0       1       t       t2      t3      ; ...       % for xf
    0       0       0       0       0       1       0       0       ; ...       % for vx0
    0       0       0       0       0       1       2*t     3*t2    ; ...       % for vxf
    1       0       0       0       0       0       0       0       ; ...       % for theta0
    1       t       t2      t3      0       0       0       0       ; ...       % for thetaf
    0       1       0       0       0       0       0       0       ; ...       % for omega0
    0       1       2*t     3*t2    0       0       0       0       ; ...       % for omegaf
    ];
    
C         = M\V;

%% for y & theta
V2         = [ y0 ; yf ; vy0 ; vyf ; theta0 ; thetaf ; omega0 ; omegaf ];

t           = tfinal;
t2          = t^2;
t3          = t^3;
t4          = t^4;
t5          = t^5;

M2          = [ ...
    0       0       0       0       1       0       0       0       ; ...       % for y0
    0       0       0       0       1       t       t2      t3      ; ...       % for yf
    0       0       0       0       0       1       0       0       ; ...       % for vy0
    0       0       0       0       0       1       2*t     3*t2    ; ...       % for vyf
    1       0       0       0       0       0       0       0       ; ...       % for theta0
    1       t       t2      t3      0       0       0       0       ; ...       % for thetaf
    0       1       0       0       0       0       0       0       ; ...       % for omega0
    0       1       2*t     3*t2    0       0       0       0       ; ...       % for omegaf
    ];
    
C2         = M2\V2;


disp('Done')

Ctheta  = C(1:4);
Cx      = C(5:8);
Cy      = C2(5:8);

return
