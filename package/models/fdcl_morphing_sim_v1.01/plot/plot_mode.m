%% initialize
close all
clear
clc

d2r = pi/180; % angle conversion parameter
r2d = 180/pi; % angle conversion parameter

%% load data
spec_data
aero_data

%% trimming
h0 = 300; % initial altitude [m]
VT0  = 20; % initial true airspeed [m/s]
eta0 = zeros(2,1); % initial morphing parameter

alp_guess = 0; % guess for trimmed AoA [rad]
delt_guess = 0; % guess for trimmed throttle
dele_guess = 0; % guess for trimmed elevator
z_guess = [alp_guess;delt_guess;dele_guess]; % guess for trimmed state and control

[x_trim,u_trim,alp_trim] = trim_calc(z_guess,VT0,h0,eta0); % trim calculation

disp('-----------------------------------------------------------')
disp('Trim (straight and level flight with a constant airspeed)\n')
fprintf('AoA      [deg] = %f\n',alp_trim*r2d)
fprintf('Throttle [   ] = %f\n',u_trim(1))
fprintf('Elevator [deg] = %f\n',u_trim(3)*r2d)
disp('-----------------------------------------------------------')
f = state_eq(x_trim,u_trim,eta0); % state derivative
fprintf('Velocity Rate Trim Error: %f m/s^2\n',norm(f(1:3)))
fprintf('Angular Velocity Rate Trim Error: %f deg/s^2\n',rad2deg(norm(f(4:6))))
disp('-----------------------------------------------------------')

[~,~,~,rho0] = atmosisa(h0);
Re = rho0*VT0*SPEC.cbar/(1.789*1e-5); % Reynolds number (https://en.wikipedia.org/wiki/Standard_sea_level)

%% linearization
bet_trim = 0;
omega_trim = x_trim(4:6);
[~,th_trim,~] = quat2angle(x_trim(7:10).');
angle_trim = [0;th_trim;0];
wind_trim = [VT0;alp_trim;bet_trim];
p0 = [0;0;-h0];

[Alon,Blon,Alat,Blat] = lin_mod([wind_trim;omega_trim;angle_trim;p0],u_trim([1 3 2 4]),eta0); % linearize
Alon(4,:) = Alon(4,:)-Alon(2,:); Blon(4,:) = Blon(4,:)-Blon(2,:); % state transformation from theta to gamma
x_trim_lon = [VT0;alp_trim;omega_trim(2);th_trim-alp_trim]; % longitudinal state trim value
x_trim_lat = zeros(4,1); % lateral state trim value

%%
[Wn_lon,zeta_lon] = damp(Alon);
damp(Alon)

figure('innerposition',[500 500 500 450])

h = pzplot(ss(Alon,Blon,eye(4),zeros(4,2)),'r');
grid on, title('Longitudinal Mode')
axis equal