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
hf = figure('position',[-1100 -400 700 1106]);
ha = tight_subplot(9,1,[.01 .01],[.05 .05],[.1 .1]);

axes(ha(1))
plot(t,alp_lin,'k',t,r,'r--');
grid on
box on
ylim([-0.3 1.5])
ylabel('Trimmed AoA, deg')
legend({'$\hat{\alpha}$','r'},'Interpreter','latex','location','northeast')

axes(ha(2))
plot(t,alp,'k',t,r+alp_trim,'r--',t,alp_trim,'k-.');
grid on
box on
ylabel('AoA, deg')
legend('\alpha','r','\alpha_{eq}','location','best')
ylim([2 5.5])

axes(ha(3))
plot(t,q,'k');
grid on
box on
ylabel('Pitch Rate, deg/s')
legend('q','location','northeast')
ylim([-12.5 9])

axes(ha(4))
plot(t,u,'k',t,u_c+u_trim,'r--');
grid on
box on
ylabel('Elevator, deg')
legend('\delta_e','\delta_{ec}','location','north')
ylim([-10 3])

axes(ha(5))
plot(t,eta(:,1),'k',t,eta(:,2),'k--');
grid on
box on
ylabel('Configuration')
legend('\eta_{ws}','\eta_{vs}','location','south')
ylim([-0.2 1.2])

axes(ha(6))
plot(t,V_T,'k');
grid on
box on
ylabel('Airspeed, m/s')
ylim([15 21])
legend('V_T','location','northeast')

axes(ha(7))
plot(t,theta,'k');
grid on
box on
ylabel('Pitch Angle, deg')
legend('\theta','location','northeast')

axes(ha(8))
plot(t,h,'k');
grid on
box on
ylabel('Altitude,m')
ylim([298 310])
legend('h','location','southeast')

axes(ha(9))
plot(t,A_z,'k');
grid on
box on
ylabel('Acceleration, m/s^2')
legend('A_z','location','southeast')

for ii = 1:9
    if ii == 9
        xlabel('Time, s')
    else
        set(ha(ii),'xticklabel',{[]})
    end
end

align_Ylabels(ha)

set(hf,'units','normalized')
set(hf,'papersize',[50 40])