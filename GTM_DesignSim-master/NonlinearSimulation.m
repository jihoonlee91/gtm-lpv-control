%% Initialization
% close all
% clear all
clc
% 
% setup

%% Nonlinear Simulation
vehicle='GTM_T2';
noise_on = 0;
fuelburn_on = 0;
RateBoundOn = 0; % 0 (rate-unbounded) or 1 (rate-bounded)

target            = struct('gamma',0,'tas',80); % target trim condition [knot]
MWS.StatesInp(9)  = 800; % set altitude [ft]
MWS.bias.morphing = -12.5; % set morphing parameter -> 이거 아님

% actuator bias 없애주자
% MWS.bias.aileron = 0;
% MWS.bias.elevator = 0;
% MWS.bias.rudder = 0;
% MWS.bias.throttle = 0;
% MWS.bias.morphing = 0;

loadmws(MWS,'gtm_design'); % load MWS

[MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose); % trim calculation

[tsim,xsim,ysim]=sim('gtm_design',[0 15]);

save NonlinearSimulation.mat tsim xsim ysim NED

SimulationPlot