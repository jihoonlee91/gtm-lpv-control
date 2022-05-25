%% Initialization
close all
% clear all
clc
% 
% setup

% load Linearization.mat
% load Trim.mat
% load K_lon_lpv.mat
% load K_lon_hinf.mat
% load K_lat_lpv.mat
% load K_lat_hinf.mat

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
MWS.StatesInp(9)  = 1000; % set altitude [ft]
MWS.bias.morphing = -10; % set morphing parameter -> 이거 아님

loadmws(MWS,'gtm_design'); % load MWS

[MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose); % trim calculation

% actuator bias 없애주자
% MWS.bias.aileron = 0;
% MWS.bias.elevator = 0;
% MWS.bias.rudder = 0;
% MWS.bias.throttle = 0;  
% MWS.bias.morphing = 0;

loadmws(MWS,'gtm_design'); % load MWS

[tsim,xsim,ysim]=sim('gtm_design',[0 10]);

save NonlinearSimulation.mat tsim xsim ysim NED

SimulationPlot