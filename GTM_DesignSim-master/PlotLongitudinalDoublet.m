plot(t,Az)
plot(t,alpha)
plot(t,q)
plot(t,h)
plot(t,tas)
plot(t,Az)

%%
plot(t,)

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