%
%   test_002.m
%
%   Test of robot trajectory generation
%
%   Martin Krucinski
%
%   v _002 uses DIFFERENT calculation for vy  & y ...

init_Constants;

x0      = 0.5;
y0      = 3;
theta0  = 30*deg;

xf      = 1;
yf      = 1;
thetaf  = -30*deg;%-135*deg;
vx0     = 0;
vy0     = 0;
omega0  = 0;
vxf     = 0;
vyf     = 0;
omegaf  = 0;

Robot.d	= 25*in;		% distance between wheel centerlines
Robot.R = 5/2*in;	% [m]		Robot wheel radius

R       = Robot.R;
d       = Robot.d;

tfinal      =  2.0;

[Ctheta, Cx, Cy] = calc_trajectory_v2(x0,y0,theta0,xf,yf,thetaf,vx0,vy0,omega0,vxf,vyf,omegaf,tfinal, d);

%   Flip polynomials for use with polyval function
% Ctheta_val      = flipud(Ctheta);
% Cx_val          = flipud(Cx);
% Cy_val          = flipud(Cy);

%   Note! No fliipud requred with calc_trajectory_v2, coefficient vector
%   has been flipped in that function
Ctheta_val      = (Ctheta);
Cx_val          = (Cx);
Cy_val          = (Cy);

%   Calculate velocity polynomials
Comega_val      = polyder(Ctheta_val);
Cvx_val         = polyder(Cx_val);
Cvy_val         = polyder(Cy_val);

%   Calculate trajectories
Ts          = 0.010;
all_t       = 0:Ts:tfinal;


all_theta   = polyval(Ctheta_val, all_t);
all_x       = polyval(Cx_val, all_t);
%all_y       = polyval(Cy_val, all_t);


all_omega   = polyval(Comega_val, all_t);
all_vx      = polyval(Cvx_val, all_t);
%all_vy      = polyval(Cvy_val, all_t);
all_vy      = all_vx.*tan(all_theta);

all_y       = y0 + cumsum(all_vy)*Ts;


%%
f1=figure;
title('Robot Trajectory')
set(gcf, 'DefaultLineLineWidth', 3);

subplot(311)
plot(all_t, all_x)
ylabel('x  [m]')
grid on
hold on
plot(0, x0, 'bo')
plot(tfinal, xf, 'bo')

subplot(312)
set(gca, 'DefaultLineLineWidth', 3);
plot(all_t, all_y)
ylabel('y [m]')
grid on
hold on
plot(0, y0, 'bo')
plot(tfinal, yf, 'bo')

subplot(313)
plot(all_t, all_theta)
ylabel('theta [rad]')
xlabel('t [s]')
grid on
hold on
plot(0, theta0, 'bo')
plot(tfinal, thetaf, 'bo')

%%
f2=figure;
title('Robot Velocities')
set(gcf, 'DefaultLineLineWidth', 3);

subplot(311)
plot(all_t, all_vx)
ylabel('vx  [m/s]')
grid on
hold on
plot(0, vx0, 'bo')
plot(tfinal, vxf, 'bo')

subplot(312)
set(gca, 'DefaultLineLineWidth', 3);
plot(all_t, all_vy)
ylabel('vy [m/s]')
grid on
hold on
plot(0, vy0, 'bo')
plot(tfinal, vyf, 'bo')

subplot(313)
plot(all_t, all_omega)
ylabel('omega [rad/s]')
grid on
hold on
plot(0, omega0, 'bo')
plot(tfinal, omegaf, 'bo')
xlabel('t [s]')


%   Now, ideally we need polynomials for vR & vL
%   For now, just calculate the velocity points

all_v_Fwd       = sqrt(all_vx.^2+all_vy.^2);

all_v_R         = all_v_Fwd - 1/2 * all_omega * d;
all_v_L         = all_v_Fwd + 1/2 * all_omega * d;
all_omega_R     = all_v_R / R;
all_omega_L     = all_v_L / R;

%%
f3=figure;
title('Wheel Linear Velocities')
set(gcf, 'DefaultLineLineWidth', 3);

subplot(211)
plot(all_t, all_v_R)
ylabel('Right Wheel Linear Velocity [m/s]')
grid on

subplot(212)
plot(all_t, all_vx)
plot(all_t, all_v_L)
ylabel('Left Wheel Linear Velocity [m/s]')
xlabel('t [s]')
grid on

%%
f3=figure;
title('Wheel Angular Velocities')
set(gcf, 'DefaultLineLineWidth', 3);

subplot(211)
plot(all_t, all_omega_R)
ylabel('Right Wheel Velocity [rad/s]')
grid on

subplot(212)
plot(all_t, all_vx)
plot(all_t, all_omega_L)
ylabel('Left Wheel Velocity [rad/s]')
xlabel('t [s]')
grid on

