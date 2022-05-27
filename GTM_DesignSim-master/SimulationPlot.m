close all

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
p   = ysim(:,10)*180/pi; % [rad/s] -> [deg/s]
q   = ysim(:,11)*180/pi; % [rad/s] -> [deg/s]
r   = ysim(:,12)*180/pi; % [rad/s] -> [deg/s]
lat = ysim(:,13)*180/pi; % [rad] -> [deg]
lon = ysim(:,14)*180/pi; % [rad] -> [deg]
alt = ysim(:,15); % [ft]
phi = ysim(:,16)*180/pi; % [rad] -> [deg]
the = ysim(:,17)*180/pi; % [rad] -> [deg]
psi = ysim(:,18)*180/pi; % [rad] -> [deg]
Ax  = ysim(:,19); % [G]
Ay  = ysim(:,20); % [G]
Az  = ysim(:,21); % [G]
pN = NED(:,1); % [ft]
pE = NED(:,2); % [ft]

dele_t = control_c(:,1);
dele_c = control_c(:,3);
etaL_c = control_c(:,7)-12.5;
etaR_c = control_c(:,8)-12.5;
etaS_c = (etaR_c+etaL_c)/2;
etaA_c = (etaR_c-etaL_c)/2;
delr_c = control_c(:,9);

dele = control(:,1);
etaL = control(:,5);
etaR = control(:,6);
etaS = (etaR+etaL)/2;
etaA = (etaR-etaL)/2;
delr = control(:,7);
delt = control(:,19);

delt_command = command(:,1);
Az_c = command(:,2);
bet_c = command(:,3);
phi_c = command(:,4);
etaS_command = command(:,5);

%% Longitudinal state
figure('InnerPosition',[100 100 500 600])
subplot(511)
plot(t,Az,t,Az_c), grid on, grid minor
xlabel('Time (sec)'), ylabel('A_z (G)')
legend('Az','Az_c')

subplot(512)
plot(t,alp), grid on, grid minor
xlabel('Time (sec)'), ylabel('\alpha (deg)')

subplot(513)
plot(t,q), grid on, grid minor
xlabel('Time (sec)'), ylabel('q (deg/s)')

subplot(514)
plot(t,tas), grid on, grid minor
xlabel('Time (sec)'), ylabel('V_T (knot)')

subplot(515)
plot(t,alt), grid on, grid minor
xlabel('Time (sec)'), ylabel('h (ft)')

exportgraphics(gcf,'LongitudinalState.pdf')

%% Lateral state
figure('InnerPosition',[100 100 500 600])
subplot(511)
plot(t,bet,t,bet_c), grid on, grid minor
xlabel('Time (sec)'), ylabel('\beta (deg)')
legend('\beta','\beta_c')

subplot(512)
plot(t,p), grid on, grid minor
xlabel('Time (sec)'), ylabel('p (deg/s)')

subplot(513)
plot(t,r), grid on, grid minor
xlabel('Time (sec)'), ylabel('r (deg/s)')

subplot(514)
plot(t,phi,t,phi_c), grid on, grid minor
xlabel('Time (sec)'), ylabel('\phi (deg)')
legend('\phi','\phi_c')

subplot(515)
plot(t,psi), grid on, grid minor
xlabel('Time (sec)'), ylabel('\psi (deg)')

exportgraphics(gcf,'LateralState.pdf')

%% Control input
figure('InnerPosition',[100 100 500 600])
subplot(311)
plot(t,dele), grid on, grid minor
xlabel('Time (sec)'), ylabel('\deltae (deg)')

subplot(312)
plot(t,etaL,t,etaR,t,etaS,t,etaA), grid on, grid minor
xlabel('Time (sec)'), ylabel('\eta (%)')
legend('\etaL','\etaR','\etaS','\etaA')

subplot(313)
plot(t,delr), grid on, grid minor
xlabel('Time (sec)'), ylabel('\deltar (deg)')

exportgraphics(gcf,'ControlInput.pdf')

%% Full
% figure
% subplot(221)
% plot(t,u,t,v,t,w)
% xlabel('Time [sec]')
% ylabel('Velocity [ft/s]')
% legend('u','v','w')
% grid on, grid minor
% 
% subplot(222)
% plot(t,p*180/pi,t,q*180/pi,t,r*180/pi)
% xlabel('Time [sec]')
% ylabel('Angular Velocity [deg/s]')
% legend('p','q','r')
% grid on, grid minor
% 
% subplot(223)
% plot(t,phi*180/pi,t,the*180/pi,t,psi*180/pi)
% xlabel('Time [sec]')
% ylabel('Euler Angle [deg]')
% legend('\phi','\theta','\psi')
% grid on, grid minor
% 
% subplot(224)
% plot(t,pN,t,pE,t,alt)
% xlabel('Time [sec]')
% ylabel('Position [ft/s]')
% legend('North','East','Up')
% grid on, grid minor

%% Acc
% figure
% plot(t,Ax,t,Ay,t,Az)
% xlabel('Time [sec]')
% ylabel('Acceleration [ft/s^2]')
% legend('Ax','Ay','Az')
% grid on, grid minor
