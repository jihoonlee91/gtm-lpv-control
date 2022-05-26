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
MWS.StatesInp(9)  = 3000; % set altitude [ft]
MWS.bias.morphing = -12.5; % set morphing parameter -> 이거 아님

loadmws(MWS,'gtm_design'); % load MWS

[MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose); % trim calculation

loadmws(MWS,'gtm_design'); % load MWS
%%
[tsim,xsim,ysim]=sim('gtm_design',[0 10]);

save NonlinearSimulation.mat tsim xsim ysim NED

SimulationPlot

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

%% Full
figure
subplot(221)
plot(t,u,t,v,t,w)
xlabel('Time [sec]')
ylabel('Velocity [ft/s]')
legend('u','v','w')
grid on, grid minor

subplot(222)
plot(t,p*180/pi,t,q*180/pi,t,r*180/pi)
xlabel('Time [sec]')
ylabel('Angular Velocity [deg/s]')
legend('p','q','r')
grid on, grid minor

subplot(223)
plot(t,phi*180/pi,t,the*180/pi,t,psi*180/pi)
xlabel('Time [sec]')
ylabel('Euler Angle [deg]')
legend('\phi','\theta','\psi')
grid on, grid minor

subplot(224)
plot(t,pN,t,pE,t,alt)
xlabel('Time [sec]')
ylabel('Position [ft/s]')
legend('North','East','Up')
grid on, grid minor

%% Acc
figure
plot(t,Ax,t,Ay,t,Az)
xlabel('Time [sec]')
ylabel('Acceleration [ft/s^2]')
legend('Ax','Ay','Az')
grid on, grid minor

%% Longitudinal plot
% tas alp q the h Ax Az
% dele delt
% figure
% subplot(321)
% plot(t,tas)
% subplot(321)
% plot(t,alp), hold on
% plot(t,the)
% subplot(321)
% plot(t,q)
% subplot(321)
% plot(t,h)
% subplot(321)
% plot(t,Ax)
% plot(t,Az)
% subplot(321)
% plot(t,)
