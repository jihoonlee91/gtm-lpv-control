%% Initialization
close all
clear all
clc

% Model basic setup
setup

%%
load lpv_design_data.mat
% load trim_data.mat
% load linear_model_data.mat
% load lon_hinf.mat
% load lon_lpv.mat
% load lat_hinf.mat
% load lat_lpv.mat
% load lon_ex_hinf.mat
% load lon_ex_lpv.mat
% load lat_ex_hinf.mat
% load lat_ex_lpv.mat

knots2fps = 1.68781; % knots to feet per seconds
fps2knots = 1/knots2fps; % knots to feet per seconds
m2ft      = 3.28084; % meters to feet
d2r       = pi/180; % degrees to radians
r2d       = 180/pi;
g         = 32.174; % gravitational acceleration [ft/s^2]

%% Nonlinear simulation
etas0 = -12.5; % [%]
alt0  = 1000; % [ft]
% sas0  = 80; % [kt]
% tas0  = sas2tas(sas0,alt0); % [kt]
tas0  = 75; % [kt]
gam0  = 0; % [deg]

MWS               = init_design('GTM',1);
MWS.bias.morphing = etas0; % set morphing parameter
MWS.StatesInp(9)  = alt0; % set altitude [ft]
target            = struct('gamma',gam0,'tas',tas0); % start trim condition target [knot]
% MWS.bias.aileron = 0;
% MWS.bias.rudder = 0;
loadmws(MWS,'gtm_design'); % load MWS
set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','on') % control off
set_param('gtm_design/Guidance','commented','on')
set_param('gtm_design/Monitor','commented','on')
set_param('gtm_design','MinStep','auto')
[MWS,Xtrim,Trimcond,Err] = trimgtm(target,'elev',1); % trim calculation
% MWS.StatesInp(4) = 1*d2r; % p [rad/s]
% MWS.StatesInp(5) = 1*d2r; % q [rad/s]
% MWS.StatesInp(6) = 1*d2r; % r [rad/s]
% MWS.StatesInp(10) = 1*d2r; % phi [rad/s]
% MWS.StatesInp(11) = 1*d2r; % theta [rad/s]
% MWS.StatesInp(12) = 1*d2r; % psi [rad/s] 
loadmws(MWS,'gtm_design'); % load MWS

set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','off') % control on
set_param('gtm_design/Guidance','commented','off')
set_param('gtm_design/Monitor','commented','off')
set_param('gtm_design','MaxStep','auto')
set_param('gtm_design','RelTol','1e-2')
% set_param('gtm_design','MinStep','0.0049')
set_param('gtm_design','MinStep','0.003')

% FlightMode
% Step: 1.0 (zero), 1.1 (normal acceleration), 1.2 (sideslip), 1.3 (roll rate)
% Maneuver: 2.1 (push-over and pull-up), 2.2 (high-g turn)
% Step: 3.0 (zero), 3.1 (airspeed), 3.2 (altitude), 3.3 (sideslip), 3.4 (roll)
% Trajectory: 4.1 (waypoint following), 4.2 (orbit following)

% ControlMode
% Off: 0
% Hinf CAS: 1
% LPV CAS: 2
% Hinf Autopilot: 3
% LPV Autopilot: 4

FlightMode = 4.2;
ControlMode = 4;
deltc = MWS.bias.throttle;
% deltc = 100;
if FlightMode < 4
    set_param('gtm_design/Guidance/Autopilot Step','commented','off')
    set_param('gtm_design/Guidance/Autopilot Trajectory Tracking','commented','on')
    set_param('gtm_design/Guidance/Heading Control','commented','on')
else
    set_param('gtm_design/Guidance/Autopilot Step','commented','on')
    set_param('gtm_design/Guidance/Autopilot Trajectory Tracking','commented','off')
    set_param('gtm_design/Guidance/Heading Control','commented','off')
end
Waypoints = [0 0 -alt0;
             3000 0 -alt0;
             3000 3000 -alt0;
             0 3000 -alt0;
             0 0 -alt0];

[tsim,xsim,ysim]=sim('gtm_design',[0 300]);