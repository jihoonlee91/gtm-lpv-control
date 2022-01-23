%%

set(groot,'DefaultAxesFontSize',10)
set(groot,'DefaultLineLineWidth',2)

% close all
d2r = pi/180; r2d = 180/pi;

t =  simOut.get('t');
tmax = t(end); dt = t(2)-t(1);
cmd = simOut.get('cmd');
delc = reshape(simOut.get('delc'),[4 tmax/dt+1]).';
del = reshape(simOut.get('del'),[4 tmax/dt+1]).';
state16 = reshape(simOut.get('state16'),[16 tmax/dt+1]).';
eta = simOut.get('eta');
quat = state16(:,7:10);
pos = state16(:,11:13);
x = pos(:,1);
y = pos(:,2);
z = pos(:,3);
[ps,th,ph] = quat2angle(quat);
[~,SoS,~,~] = atmosisa(-state16(:,13));
SoS = SoS.';
for ii = 1:length(t)
    UVWi(ii,:) = quat2dcm(quat(ii,:)).'*state16(ii,1:3).';
end
gamma = -atan2(UVWi(:,3),sqrt(sum(UVWi(:,1:2).^2,2)));

%%

% set(0,'defaultFigureVisible','off')

%%

figure('InnerPosition',[50 50 1000 900])

subplot(4,2,1),plot(t,state16(:,14),t,cmd(:,1),'--','linewidth',2)
grid on
xlabel('Time [s]')
ylabel('V_T')

subplot(4,2,2), plot(t,state16(:,15)*r2d,'linewidth',2)
grid on
xlabel('Time [s]')
ylabel('\alpha [deg]')

subplot(4,2,3), plot(t,state16(:,5)*r2d,'linewidth',2)
grid on
ylabel('q [deg/s]')
xlabel('Time [s]')

subplot(4,2,4), plot(t,gamma*r2d,t,cmd(:,2)*r2d,'--','linewidth',2)
grid on
ylabel('\gamma [deg]')
xlabel('Time [s]')

% subplot(4,2,5), plot(t,state16(:,16)*r2d,t,cmd(:,3)*r2d,'--','linewidth',2)
% grid on
% xlabel('Time [s]')
% ylabel('\beta [deg]')
% 
% subplot(4,2,6), plot(t,state16(:,4)*r2d,'linewidth',2)
% grid on
% ylabel('p [deg/s]')
% xlabel('Time [s]')
% 
% subplot(4,2,7), plot(t,state16(:,6)*r2d,'linewidth',2)
% grid on
% ylabel('r [deg/s]')
% xlabel('Time [s]')
% 
% subplot(4,2,8), plot(t,ph*r2d,t,cmd(:,4)*r2d,'--','linewidth',2)
% grid on
% ylabel('\phi [deg]')
% xlabel('Time [s]')

subplot(4,2,5), plot(t,del(:,1),'linewidth',2)
grid on
ylabel('\delta_t')
xlabel('Time [s]')
ylim([-0.1 1.1])

% subplot(4,2,10), plot(t,del(:,2)*r2d,'linewidth',2)
% grid on
% ylabel('\delta_a [deg]')
% xlabel('Time [s]')
% ylim([SPEC.dela_ll*1.1 SPEC.dela_ul*1.1]*r2d)

subplot(4,2,6), plot(t,del(:,3)*r2d,'linewidth',2)
grid on
ylabel('\delta_e [deg]')
xlabel('Time [s]')
ylim([SPEC.dele_ll*1.1 SPEC.dele_ul*1.1]*r2d)

% subplot(4,2,12), plot(t,del(:,4)*r2d,'linewidth',2)
% grid on
% ylabel('\delta_r [deg]')
% xlabel('Time [s]')
% ylim([SPEC.delr_ll*1.1 SPEC.delr_ul*1.1]*r2d)

subplot(4,2,7)
plot(t,eta(:,1),'-.','linewidth',2), hold on
plot(t,eta(:,2),'--','linewidth',2)
grid on
xlabel('Time [s]')
ylabel('\eta')
ylim([-0.1 1.1])
legend('\eta_1','\eta_2')

subplot(4,2,8), plot(t,-z,'linewidth',2)
grid on
xlabel('Time [s]')
ylabel('h [m]')

% subplot(4,2,15), plot(t,-z,'linewidth',2)
% grid on
% xlabel('Time [s]')
% ylabel('h [m]')
% 
% subplot(4,2,16), plot(t,ps*r2d,'linewidth',2)
% grid on
% xlabel('Time [s]')
% ylabel('\psi [deg]')

% fig = gcf;
% fig.PaperUnits = 'inches';
% fig.PaperSize = [10 7];
% fig.PaperPosition = [0 0 fig.PaperSize];
% print('fig_plot_pushover','-depsc')
% print('fig_plot_pushover','-dpdf')
% winopen('fig_plot_pushover.pdf')

%%

figure('InnerPosition',[60 350 1800 200])

for ii = 1:length(t)
    ENU(:,ii) = [cos(-pi/2) -sin(-pi/2) 0; sin(-pi/2) cos(-pi/2) 0; 0 0 1]*[1 0 0; 0 cos(pi) -sin(pi); 0 sin(pi) cos(pi)]*[x(ii); y(ii); z(ii)];
end
trajectory3(-ENU(2,:),ENU(1,:),ENU(3,:),th,ph,ps-pi/2,1/25,(length(t)-1)/t(end)*5,'boeing_747');
xlabel('North [m]')
ylabel('East [m]')
zlabel('Altitude [m]')
view(0,0)

% fig = gcf;
% fig.PaperUnits = 'inches';
% fig.PaperSize = [10 6];
% fig.PaperPosition = [0 0 fig.PaperSize];
% print('fig_plot_pushover_3D','-depsc')
% print('fig_plot_pushover_3D','-dpdf','-r600')
% winopen('fig_plot_pushover_3D.pdf')