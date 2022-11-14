set(0,'DefaultLineLineWidth',1)
tlim = 3;
%% an step state response
figure('units','centimeters','position',[0 7 13 20])
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lon_ex_hinf_point = CL_lon_ex_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_ex_lpv_point = CL_lon_ex_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [y,t] = step(CL_lon_ex_hinf_point,tlim);
    subplot(4,1,1), plot(t,y(:,1),'color',[0 0.4470 0.7410]), ylabel('Angle of Attack [deg]'), xticklabels('')
    subplot(4,1,2), plot(t,y(:,2),'color',[0 0.4470 0.7410]), ylabel('Pitch Rate [deg/s]'), xticklabels('')
    subplot(4,1,3), plot(t,-y(:,3),'color',[0 0.4470 0.7410]), ylabel('Normal Acceleration [g]'), xticklabels('')
    subplot(4,1,4), plot(t,y(:,4),'color',[0 0.4470 0.7410]), xlabel('Time [sec]'), ylabel('Elevator [deg]')
    if (i==1)&&(j==1)&&(k==1)
        subplot(4,1,1), hold on, grid on, grid minor
        subplot(4,1,2), hold on, grid on, grid minor 
        subplot(4,1,3), hold on, grid on, grid minor
        subplot(4,1,4), hold on, grid on, grid minor
    end
    [y,t] = step(CL_lon_ex_lpv_point,tlim);
    subplot(4,1,1), plot(t,y(:,1),'--','color',[0.8500 0.3250 0.0980]), ylabel('Angle of Attack [deg]'), xticklabels(''), legend('H_\infty','LPV','location','best'), xlim([0 tlim])
    subplot(4,1,2), plot(t,y(:,2),'--','color',[0.8500 0.3250 0.0980]), ylabel('Pitch Rate [deg/s]'), xticklabels(''), legend('H_\infty','LPV','location','best'), xlim([0 tlim])
    subplot(4,1,3), plot(t,-y(:,3),'--','color',[0.8500 0.3250 0.0980]), ylabel('Normal Acceleration [g]'), xticklabels('')
    plot(t,y(:,1,1)*0+2,':','color',[0.9290 0.6940 0.1250]), legend('H_\infty','LPV','Reference','location','best'), xlim([0 tlim])
    subplot(4,1,4), plot(t,y(:,4),'--','color',[0.8500 0.3250 0.0980]), xlabel('Time [sec]'), ylabel('Elevator [deg]'), legend('H_\infty','LPV','location','best'), xlim([0 tlim])
end
end
end
exportgraphics(gcf,'figure/lon_ex_step_lin_an.png','resolution',300)
