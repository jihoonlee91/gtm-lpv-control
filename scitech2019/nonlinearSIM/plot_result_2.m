%%
close all
% set(groot,'DefaultAxesFontSize',10)
set(groot,'DefaultAxesFontSize',9.5)
set(groot,'DefaultLineLineWidth',1)

r2d = 180/pi;

t = 0:dt:tf;
r = simOut.get('r')*r2d;
eta = simOut.get('eta');
u_c = simOut.get('u_c')*r2d;
u_trim = simOut.get('u_trim')*r2d;
u = simOut.get('u')*r2d;
delt_trim = simOut.get('delt_trim');
alp_trim = simOut.get('alp_trim')*r2d;
x = simOut.get('x');
y = simOut.get('y');

V_T = x(:,1);
alp = x(:,2)*r2d;
q = x(:,3)*r2d;
theta = x(:,4)*r2d;
h = x(:,5);

alp_lin = y(:,1)*r2d;
A_z = y(:,3);

%%
hf = figure('position',[-1100 -400 700 700]);
ha = tight_subplot(4,2,[.04 .08],[.1 .05],[.07 .07]);

% axes(ha(1))
% plot(t,alp_lin,'k',t,r,'r--');
% grid on
% box off
% ylim([-0.3 1.5])
% ylabel('Trimmed AoA, deg')
% legend({'$\hat{\alpha}$','r'},'Interpreter','latex','location','northeast')

axes(ha(1))
plot(t,alp,'k',t,r+alp_trim,'r--',t,alp_trim,'k-.');
grid on
box off
ylabel('AoA, deg')
legend('\alpha','r','\alpha_{eq}','location','best')
% ylim([2 5.5])

axes(ha(2))
plot(t,q,'k');
grid on
box off
ylabel('Pitch Rate, deg/s')
% ylim([-12.5 9])

axes(ha(3))
plot(t,u,'k');
grid on
box off
ylabel('Elevator, deg')
% ylim([-10 3])

axes(ha(4))
plot(t,eta(:,1),'k',t,eta(:,2),'k--');
grid on
box off
ylabel('Configuration')
% ylim([-0.2 1.2])

axes(ha(5))
plot(t,V_T,'k');
grid on
box off
ylabel('Airspeed, m/s')
% ylim([15 21])

axes(ha(6))
plot(t,theta,'k');
grid on
box off
ylabel('Pitch Angle, deg')

axes(ha(7))
plot(t,h,'k');
grid on
box off
ylabel('Altitude,m')
% ylim([298 310])
legend('h','location','southeast')

axes(ha(8))
plot(t,A_z,'k');
grid on
box off
ylabel('Acceleration, m/s^2')

for ii = 1:8
    axes(ha(ii))
    if ii >= 7
        xlabel('Time, s')
%     else
%         set(ha(ii),'xticklabel',{[]})
    end
end

% align_Ylabels(ha(1:4))

set(hf,'units','normalized')
set(hf,'papersize',[50 40])