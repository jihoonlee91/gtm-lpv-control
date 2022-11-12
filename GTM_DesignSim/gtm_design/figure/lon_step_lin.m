set(0,'DefaultLineLineWidth',1)
%% tas step state response
figure('units','centimeters','position',[0 2 13 20])
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lon_hinf_point = CL_lon_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_lpv_point = CL_lon_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [y,t] = step(CL_lon_hinf_point,30);
    subplot(5,1,1), plot(t,y(:,1,1),'--','color',[0 0.4470 0.7410]), ylabel('True Airspeed [kt]'), xticklabels('')
    subplot(5,1,2), plot(t,y(:,2,1),'--','color',[0 0.4470 0.7410]), ylabel('Angle of Attack [deg]'), xticklabels('')
    subplot(5,1,3), plot(t,y(:,3,1),'--','color',[0 0.4470 0.7410]), ylabel('Pitch Rate [deg/s]'), xticklabels('')
    subplot(5,1,4), plot(t,y(:,4,1),'--','color',[0 0.4470 0.7410]), ylabel('Pitch Angle [deg]'), xticklabels('')
    subplot(5,1,5), plot(t,y(:,5,1),'--','color',[0 0.4470 0.7410]), xlabel('Time [sec]'), ylabel('Altitude [ft]')
    if (i==1)&&(j==1)&&(k==1)
        subplot(5,1,1), hold on, grid on, grid minor
        subplot(5,1,2), hold on, grid on, grid minor 
        subplot(5,1,3), hold on, grid on, grid minor
        subplot(5,1,4), hold on, grid on, grid minor
        subplot(5,1,5), hold on, grid on, grid minor
    end
    [y,t] = step(CL_lon_lpv_point,30);
    subplot(5,1,1), plot(t,y(:,1,1),'color',[0.8500 0.3250 0.0980]), ylabel('True Airspeed [kt]'), xticklabels('')
    plot(t,y(:,1,1)*0+10,':','color',[0.9290 0.6940 0.1250]), legend('H_\infty','LPV','Reference','location','best'), xlim([0 30])
    subplot(5,1,2), plot(t,y(:,2,1),'color',[0.8500 0.3250 0.0980]), ylabel('Angle of Attack [deg]'), xticklabels(''), legend('H_\infty','LPV','location','best'), xlim([0 30])
    subplot(5,1,3), plot(t,y(:,3,1),'color',[0.8500 0.3250 0.0980]), ylabel('Pitch Rate [deg/s]'), xticklabels(''), legend('H_\infty','LPV','location','best'), xlim([0 30])
    subplot(5,1,4), plot(t,y(:,4,1),'color',[0.8500 0.3250 0.0980]), ylabel('Pitch Angle [deg]'), xticklabels(''), legend('H_\infty','LPV','location','best'), xlim([0 30])
    subplot(5,1,5), plot(t,y(:,5,1),'color',[0.8500 0.3250 0.0980]), xlabel('Time [sec]'), ylabel('Altitude [ft]')
    plot(t,y(:,1,1)*0+0,':','color',[0.9290 0.6940 0.1250]), legend('H_\infty','LPV','Reference','location','best'), xlim([0 30])
end
end
end
exportgraphics(gcf,'figure/tas_step_lin_state.png','resolution',300)

%% tas step input response
figure('units','centimeters','position',[2*2 2 13 20])
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lon_hinf_point = CL_lon_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_lpv_point = CL_lon_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [y,t] = step(CL_lon_hinf_point,30);
    subplot(2,1,1), plot(t,y(:,6,1),'--','color',[0 0.4470 0.7410]), ylabel('Throttle [%]'), xticklabels('')
    subplot(2,1,2), plot(t,y(:,7,1),'--','color',[0 0.4470 0.7410]), xlabel('Time [sec]'), ylabel('Elevator [deg]')
    if (i==1)&&(j==1)&&(k==1)
        subplot(2,1,1), hold on, grid on, grid minor
        subplot(2,1,2), hold on, grid on, grid minor 
    end
    [y,t] = step(CL_lon_lpv_point,30);
    subplot(2,1,1), plot(t,y(:,6,1),'color',[0.8500 0.3250 0.0980]), ylabel('Throttle [kt]'), xticklabels(''), legend('H_\infty','LPV','location','best'), xlim([0 30])
    subplot(2,1,2), plot(t,y(:,7,1),'color',[0.8500 0.3250 0.0980]), xlabel('Time [sec]'), ylabel('Elevator [deg]'), legend('H_\infty','LPV','location','best'), xlim([0 30])
end
end
end
exportgraphics(gcf,'figure/tas_step_lin_input.png','resolution',300)

%% alt step state response
figure('units','centimeters','position',[4*2 2 13 20])
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lon_hinf_point = CL_lon_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_lpv_point = CL_lon_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [y,t] = step(CL_lon_hinf_point,30);
    subplot(5,1,1), plot(t,y(:,1,2),'--','color',[0 0.4470 0.7410]), ylabel('True Airspeed [kt]'), xticklabels('')
    subplot(5,1,2), plot(t,y(:,2,2),'--','color',[0 0.4470 0.7410]), ylabel('Angle of Attack [deg]'), xticklabels('')
    subplot(5,1,3), plot(t,y(:,3,2),'--','color',[0 0.4470 0.7410]), ylabel('Pitch Rate [deg/s]'), xticklabels('')
    subplot(5,1,4), plot(t,y(:,4,2),'--','color',[0 0.4470 0.7410]), ylabel('Pitch Angle [deg]'), xticklabels('')
    subplot(5,1,5), plot(t,y(:,5,2),'--','color',[0 0.4470 0.7410]), xlabel('Time [sec]'), ylabel('Altitude [ft]')
    if (i==1)&&(j==1)&&(k==1)
        subplot(5,1,1), hold on, grid on, grid minor
        subplot(5,1,2), hold on, grid on, grid minor 
        subplot(5,1,3), hold on, grid on, grid minor
        subplot(5,1,4), hold on, grid on, grid minor
        subplot(5,1,5), hold on, grid on, grid minor
    end
    [y,t] = step(CL_lon_lpv_point,30);
    subplot(5,1,1), plot(t,y(:,1,2),'color',[0.8500 0.3250 0.0980]), ylabel('True Airspeed [kt]'), xticklabels(''), legend('H_\infty','LPV','location','best'), xlim([0 30])
    plot(t,y(:,1,1)*0+0,':','color',[0.9290 0.6940 0.1250]), legend('H_\infty','LPV','Reference','location','best'), xlim([0 30])
    subplot(5,1,2), plot(t,y(:,2,2),'color',[0.8500 0.3250 0.0980]), ylabel('Angle of Attack [deg]'), xticklabels(''), legend('H_\infty','LPV','location','best'), xlim([0 30])
    subplot(5,1,3), plot(t,y(:,3,2),'color',[0.8500 0.3250 0.0980]), ylabel('Pitch Rate [deg/s]'), xticklabels(''), legend('H_\infty','LPV','location','best'), xlim([0 30])
    subplot(5,1,4), plot(t,y(:,4,2),'color',[0.8500 0.3250 0.0980]), ylabel('Pitch Angle [deg]'), xticklabels(''), legend('H_\infty','LPV','location','best'), xlim([0 30])
    subplot(5,1,5), plot(t,y(:,5,2),'color',[0.8500 0.3250 0.0980]), xlabel('Time [sec]'), ylabel('Altitude [ft]')
    plot(t,y(:,1,1)*0+10,':','color',[0.9290 0.6940 0.1250]), legend('H_\infty','LPV','Reference','location','best'), xlim([0 30])
end
end
end
exportgraphics(gcf,'figure/alt_step_lin_state.png','resolution',300)

%% alt step input response
figure('units','centimeters','position',[6*2 2 13 20])
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lon_hinf_point = CL_lon_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_lpv_point = CL_lon_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [y,t] = step(CL_lon_hinf_point,30);
    subplot(2,1,1), plot(t,y(:,6,2),'--','color',[0 0.4470 0.7410]), ylabel('Throttle [%]'), xticklabels('')
    subplot(2,1,2), plot(t,y(:,7,2),'--','color',[0 0.4470 0.7410]), xlabel('Time [sec]'), ylabel('Elevator [deg]')
    if (i==1)&&(j==1)&&(k==1)
        subplot(2,1,1), hold on, grid on, grid minor
        subplot(2,1,2), hold on, grid on, grid minor 
    end
    [y,t] = step(CL_lon_lpv_point,30);
    subplot(2,1,1), plot(t,y(:,6,2),'color',[0.8500 0.3250 0.0980]), ylabel('Throttle [kt]'), xticklabels(''), legend('H_\infty','LPV','location','best'), xlim([0 30])
    subplot(2,1,2), plot(t,y(:,7,2),'color',[0.8500 0.3250 0.0980]), xlabel('Time [sec]'), ylabel('Elevator [deg]'), legend('H_\infty','LPV','location','best'), xlim([0 30])
end
end
end
exportgraphics(gcf,'figure/alt_step_lin_input.png','resolution',300)
