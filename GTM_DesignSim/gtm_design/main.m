%% Initialization
close all
clear all
clc

% Model basic setup
setup

%%
% lpv_design

load trim_data.mat
load linear_model_data.mat
load lon_hinf.mat
load lon_lpv.mat
load lat_hinf.mat
load lat_lpv.mat

%% Nonlinear simulation
FlightMode = 1.0;
% Step: 1.0 (zero), 1.1 (normal acceleration), 1.2 (sideslip), 1.3 (roll rate)
% Maneuver: 2.1 (push-over and pull-up), 2.2 (high-g turn)
% Step: 3.0 (zero), 3.1 (airspeed), 3.2 (altitude), 3.3 (sideslip), 3.4 (roll)
% Trajectory: 4.1 (waypoint following), 4.2 (orbit following)

ControlMode = 0;
% Off: 0
% H-Infinity: 1
% LPV: 2
% RBLPV: 3

etas0 = -12.5;
alt0  = 0;
sas0  = 80;
tas0  = sas2tas(sas0,alt0);
gam0  = 0;
MWS               = init_design('GTM',1);
MWS.bias.morphing = etas0; % set morphing parameter
MWS.StatesInp(9)  = alt0; % set altitude [ft]
target            = struct('gamma',gam0,'tas',tas0); % start trim condition target [knot]
% MWS.bias.aileron = 0;
% MWS.bias.rudder = 0;
loadmws(MWS,'gtm_design'); % load MWS
set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','on') % control off
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
set_param('gtm_design','MinStep','0.0049')
t_final = 100;
GuidanceMode = 2;
ControlMode = 3;

[tsim,xsim,ysim]=sim('gtm_design',[0 t_final]);