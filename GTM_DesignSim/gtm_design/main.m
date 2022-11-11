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