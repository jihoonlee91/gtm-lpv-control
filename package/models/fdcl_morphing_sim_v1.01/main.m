%% initialize
close all
clear
clc

%% simulation setup
tf  = 50; % simulation time [s]
dt  = 0.02; % integration time step during simulation [s]
d2r = pi/180; % angle conversion parameter
r2d = 180/pi; % angle conversion parameter

%% data
spec_data % load specification
aero_data % load aerodynamic data

%% trimming
h0 = 300; % initial altitude [m]
VT0  = 20; % initial true airspeed [m/s]
eta0 = zeros(2,1); % initial morphing parameter [span; sweep]

alp_guess = 0; % guess for trimmed AoA [rad]
delt_guess = 0; % guess for trimmed throttle
dele_guess = 0; % guess for trimmed elevator
z_guess = [alp_guess;delt_guess;dele_guess]; % guess for trimmed state and control

[x_trim,u_trim,alp_trim] = trim_calc(z_guess,VT0,h0,eta0); % trim calculation
delt_trim = u_trim(1);
dele_trim = u_trim(3);

disp('-----------------------------------------------------------')
disp('Trim (straight and level flight with a constant airspeed)')
fprintf('h    [ m ] = %f\n',h0)
fprintf('TAS  [m/s] = %f\n',VT0)
fprintf('AoA  [deg] = %f\n',alp_trim*r2d)
fprintf('delt [   ] = %f\n',delt_trim)
fprintf('dele [deg] = %f\n',dele_trim*r2d)
disp('-----------------------------------------------------------')
f = state_eq(x_trim,u_trim,eta0); % state derivative
fprintf('Velocity Rate Trim Error: %f m/s^2\n',norm(f(1:3)))
fprintf('Angular Velocity Rate Trim Error: %f deg/s^2\n',rad2deg(norm(f(4:6))))
disp('-----------------------------------------------------------')

%% lqr design
param = lqr_design(h0,VT0,eta0,alp_trim,delt_trim,dele_trim);

%% run simulation
state130 = x_trim; % initial state (used in Simulink airframe dynamics)
control0 = u_trim; % initial control (used in Simulink actuator dynamics)

VT_c = VT0; % airspeed command
gamma_c = deg2rad(0); % gamma command

bet_c = 0; % sideslip angle command (maintain to zero)
ph_c = deg2rad(0); % roll angle command

tic
simOut = sim('aircraft_model','SimulationMode','normal', ...
            'StopTime', num2str(tf), ...
            'FixedStep', num2str(dt), ...
            'SaveOutput','on','OutputSaveName','youtNew',...
            'SignalLogging','on','SignalLoggingName','logsout');
toc

plot_result