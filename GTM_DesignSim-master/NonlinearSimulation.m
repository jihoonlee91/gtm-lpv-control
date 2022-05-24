%% Initialization
close all
clear all
clc

setup
addpath(genpath('./LPVToolsV1.0/'));

%% Nonlinear Simulation
% vehicle='GTM_T2';
% noise_on = 0;
% fuelburn_on = 0;
% RateBoundOn    = 0; % 0 (rate-unbounded) or 1 (rate-bounded)
% 
% MWS.StatesInp
% target            = struct('gamma',0,'tas',80); % target trim condition [knot]
% MWS.StatesInp(9)  = 800; % set altitude [ft]
% MWS.bias.morphing = -12.5; % set morphing parameter -> 이거 아님

% actuator bias 없애주자
% MWS.bias.aileron = 0;
% MWS.bias.elevator = 0;
% MWS.bias.rudder = 0;
% MWS.bias.throttle = 0;
% MWS.bias.morphing = 0;
% 
% loadmws(MWS,'gtm_design'); % load MWS
% 
% [MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose); % trim calculation
% 
% [tsim,xsim,ysim]=sim('gtm_design',[0 15]);

% Grab state in ysim First 6 are trim outputs, next 12 are state.
% ysim=ysim(:,7:18);

%% Plot results
% set(figure(1),'Position',[20 80 900 700]);
% subplot(321),
% plot(tsim,ysim(:,1)); grid on
% title('Linear Velocity to Doublet Sequence [elevator,rudder,aileron]');
% xlabel('Time (sec)');ylabel('u (ft/sec)')
% legend('Simulation', 'Linear');
% subplot(323),
% plot(tsim,ysim(:,2)); grid on
% xlabel('Time (sec)');ylabel('v (ft/sec)')
% subplot(325),
% plot(tsim,ysim(:,3)); grid on
% xlabel('Time (sec)');ylabel('w (ft/sec)')
% 
% subplot(322),
% plot(tsim,180/pi*ysim(:,4)); grid on
% title('Angular Velocity to Doublet Sequence [elevator,rudder,aileron]');
% xlabel('Time (sec)');ylabel('p (deg/sec)')
% subplot(324),
% plot(tsim,180/pi*ysim(:,5)); grid on
% xlabel('Time (sec)');ylabel('q (deg/sec)')
% subplot(326),
% plot(tsim,180/pi*ysim(:,6)); grid on
% xlabel('Time (sec)');ylabel('r (deg/sec)')
