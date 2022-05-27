%% Initialization
close all
clear all
clc

addpath(genpath('./LPVToolsV1.0/'));

% load Linearization.mat
% load Trim.mat
% load K_lon_lpv.mat
% load K_lon_hinf.mat
% load K_lat_lpv.mat
% load K_lat_hinf.mat

setup

%% Nonlinear Simulation
vehicle      = 'GTM_T2';
noise_on     = 0;
fuelburn_on  = 0;
RateBoundOn  = 0; % 0 (rate-unbounded) or 1 (rate-bounded)
% LPVControlOn = 0; % 0 (hinf control) or 1 (lpv control)
PitchSurf      = 'elev'; % pitch surface
verbose        = 1; % trim output display off

MWS = init_design('GTM');
target            = struct('gamma',0,'tas',85); % target trim condition [knot]
MWS.StatesInp(9)  = 3000; % set altitude [ft]
MWS.bias.morphing = -12.5; % set morphing parameter -> 이거 아님
loadmws(MWS,'gtm_design'); % load MWS

[MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose); % trim calculation
loadmws(MWS,'gtm_design'); % load MWS

%%
[tsim,xsim,ysim]=sim('gtm_design',[0 10]);

save NonlinearSimulation.mat tsim xsim ysim NED control_c control command

%% Unfold
t   = tsim;
eas = ysim(:,1); % [knot]
tas = ysim(:,2); % [knot]
alp = ysim(:,3); % [deg]
bet = ysim(:,4); % [deg]
gam = ysim(:,5); % [deg]
chi = ysim(:,6); % [deg]
u   = ysim(:,7); % [ft/s]
v   = ysim(:,8); % [ft/s]
w   = ysim(:,9); % [ft/s]
p   = ysim(:,10); % [rad/s]
q   = ysim(:,11); % [rad/s]
r   = ysim(:,12); % [rad/s]
lat = ysim(:,13); % [rad]
lon = ysim(:,14); % [rad]
alt = ysim(:,15); % [ft]
phi = ysim(:,16); % [rad]
the = ysim(:,17); % [rad]
psi = ysim(:,18); % [rad]
Ax  = ysim(:,19); % [G]
Ay  = ysim(:,20); % [G]
Az  = ysim(:,21); % [G]
pN = NED(:,1); % [ft]
pE = NED(:,2); % [ft]

dele_c = control_c(:,3);
etaL_c = control_c(:,7);
etaR_c = control_c(:,8);
etaS_c = (etaR_c+etaL_c)/2;
etaA_c = (etaR_c-etaL_c)/2;
delr_c = control_c(:,9);

dele = control(:,1);
etaL = control(:,5);
etaR = control(:,6);
etaS = (etaR+etaL)/2;
etaA = (etaR-etaL)/2;
delr = control(:,7);