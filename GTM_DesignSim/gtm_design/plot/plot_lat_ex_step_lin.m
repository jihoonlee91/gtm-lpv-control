set(0,'DefaultLineLineWidth',1)
tlim = 5;
%% bet step state response
figure('units','centimeters','position',[0 2 13 20])
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lat_ex_hinf_point = CL_lat_ex_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_ex_lpv_point = CL_lat_ex_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [y,t] = step(CL_lat_ex_hinf_point,tlim);
    subplot(3,1,1), plot(t,y(:,1,1),'color',[0 0.4470 0.7410]), ylabel('Angle of Sideslip [deg]'), xticklabels('')
    subplot(3,1,2), plot(t,y(:,2,1),'color',[0 0.4470 0.7410]), ylabel('Roll Rate [deg/s]'), xticklabels('')
    subplot(3,1,3), plot(t,y(:,3,1),'color',[0 0.4470 0.7410]), xlabel('Time [sec]'), ylabel('Yaw Rate [deg/s]')
    if (i==1)&&(j==1)&&(k==1)
        subplot(3,1,1), hold on, grid on, grid minor
        subplot(3,1,2), hold on, grid on, grid minor 
        subplot(3,1,3), hold on, grid on, grid minor
    end
    [y,t] = step(CL_lat_ex_lpv_point,tlim);
    subplot(3,1,1), plot(t,y(:,1,1),'--','color',[0.8500 0.3250 0.0980]), ylabel('Angle of Sideslip [deg]'), xticklabels('')
    plot(t,y(:,1,1)*0+3,':','color',[0.9290 0.6940 0.1250]), legend('H_\infty','LPV','Reference','location','best'), xlim([0 tlim])
    subplot(3,1,2), plot(t,y(:,2,1),'--','color',[0.8500 0.3250 0.0980]), xlabel('Time [sec]'), ylabel('Roll Rate [deg/s]')
    plot(t,y(:,1,1)*0,':','color',[0.9290 0.6940 0.1250]), legend('H_\infty','LPV','Reference','location','best'), xlim([0 tlim])
    subplot(3,1,3), plot(t,y(:,3,1),'--','color',[0.8500 0.3250 0.0980]), ylabel('Yaw Rate [deg/s]'), xlabel('Time [sec]'), legend('H_\infty','LPV','location','best'), xlim([0 tlim])
end
end
end
exportgraphics(gcf,'figure/lat_ex_step_lin_bet_state.png','resolution',300)

%% bet step input response
figure('units','centimeters','position',[13 2 13 20])
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lat_ex_hinf_point = CL_lat_ex_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_ex_lpv_point = CL_lat_ex_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [y,t] = step(CL_lat_ex_hinf_point,30);
    subplot(3,1,1), plot(t,y(:,4,1),'color',[0 0.4470 0.7410]), ylabel('Aileron [deg]'), xticklabels('')
    subplot(3,1,2), plot(t,y(:,5,1),'color',[0 0.4470 0.7410]), ylabel('Rudder [deg]'), xticklabels('')
    subplot(3,1,3), plot(t,y(:,6,1),'color',[0 0.4470 0.7410]), xlabel('Time [sec]'), ylabel('Asymmetric Morphing [%]')
    if (i==1)&&(j==1)&&(k==1)
        subplot(3,1,1), hold on, grid on, grid minor
        subplot(3,1,2), hold on, grid on, grid minor 
        subplot(3,1,3), hold on, grid on, grid minor 
    end
    [y,t] = step(CL_lat_ex_lpv_point,30);
    subplot(3,1,1), plot(t,y(:,4,1),'--','color',[0.8500 0.3250 0.0980]), ylabel('Aileron [deg]'), xticklabels(''), legend('H_\infty','LPV','location','best'), xlim([0 tlim])
    subplot(3,1,2), plot(t,y(:,5,1),'--','color',[0.8500 0.3250 0.0980]), ylabel('Rudder [deg]'), xticklabels(''), legend('H_\infty','LPV','location','best'), xlim([0 tlim])
    subplot(3,1,3), plot(t,y(:,6,1),'--','color',[0.8500 0.3250 0.0980]), xlabel('Time [sec]'), ylabel('Asymmetric Morphing [%]'), legend('H_\infty','LPV','location','best'), xlim([0 tlim])
end
end
end
exportgraphics(gcf,'figure/lat_ex_step_lin_bet_input.png','resolution',300)

%% p step state response
tlim = 2;
figure('units','centimeters','position',[26 2 13 20])
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lat_ex_hinf_point = CL_lat_ex_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_ex_lpv_point = CL_lat_ex_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [y,t] = step(CL_lat_ex_hinf_point,tlim);
    subplot(3,1,1), plot(t,y(:,1,2),'color',[0 0.4470 0.7410]), ylabel('Angle of Sideslip [deg]'), xticklabels('')
    subplot(3,1,2), plot(t,y(:,2,2),'color',[0 0.4470 0.7410]), ylabel('Roll Rate [deg/s]'), xticklabels('')
    subplot(3,1,3), plot(t,y(:,3,2),'color',[0 0.4470 0.7410]), xlabel('Time [sec]'), ylabel('Yaw Rate [deg/s]')
    if (i==1)&&(j==1)&&(k==1)
        subplot(3,1,1), hold on, grid on, grid minor
        subplot(3,1,2), hold on, grid on, grid minor 
        subplot(3,1,3), hold on, grid on, grid minor
    end
    [y,t] = step(CL_lat_ex_lpv_point,tlim);
    subplot(3,1,1), plot(t,y(:,1,2),'--','color',[0.8500 0.3250 0.0980]), ylabel('Angle of Sideslip [deg]'), xticklabels('')
    plot(t,y(:,1,1)*0+0,':','color',[0.9290 0.6940 0.1250]), legend('H_\infty','LPV','Reference','location','best'), xlim([0 tlim])
    subplot(3,1,2), plot(t,y(:,2,2),'--','color',[0.8500 0.3250 0.0980]), xlabel('Time [sec]'), ylabel('Roll Rate [deg/s]')
    plot(t,y(:,1,1)*0+10,':','color',[0.9290 0.6940 0.1250]), legend('H_\infty','LPV','Reference','location','best'), xlim([0 tlim])
    subplot(3,1,3), plot(t,y(:,3,2),'--','color',[0.8500 0.3250 0.0980]), ylabel('Yaw Rate [deg/s]'), xlabel('Time [sec]'), legend('H_\infty','LPV','location','best'), xlim([0 tlim])
end
end
end
exportgraphics(gcf,'figure/lat_ex_step_lin_p_state.png','resolution',300)

%% p step input response
figure('units','centimeters','position',[39 2 13 20])
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lat_ex_hinf_point = CL_lat_ex_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_ex_lpv_point = CL_lat_ex_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [y,t] = step(CL_lat_ex_hinf_point,tlim);
    subplot(3,1,1), plot(t,y(:,4,2),'color',[0 0.4470 0.7410]), ylabel('Aileron [deg]'), xticklabels('')
    subplot(3,1,2), plot(t,y(:,5,2),'color',[0 0.4470 0.7410]), ylabel('Rudder [deg]'), xticklabels('')
    subplot(3,1,3), plot(t,y(:,6,2),'color',[0 0.4470 0.7410]), xlabel('Time [sec]'), ylabel('Asymmetric Morphing [%]')
    if (i==1)&&(j==1)&&(k==1)
        subplot(3,1,1), hold on, grid on, grid minor
        subplot(3,1,2), hold on, grid on, grid minor 
        subplot(3,1,3), hold on, grid on, grid minor 
    end
    [y,t] = step(CL_lat_ex_lpv_point,tlim);
    subplot(3,1,1), plot(t,y(:,4,2),'--','color',[0.8500 0.3250 0.0980]), ylabel('Aileron [deg]'), xticklabels(''), legend('H_\infty','LPV','location','best'), xlim([0 tlim])
    subplot(3,1,2), plot(t,y(:,5,2),'--','color',[0.8500 0.3250 0.0980]), ylabel('Rudder [deg]'), xticklabels(''), legend('H_\infty','LPV','location','best'), xlim([0 tlim])
    subplot(3,1,3), plot(t,y(:,6,2),'--','color',[0.8500 0.3250 0.0980]), xlabel('Time [sec]'), ylabel('Asymmetric Morphing [%]'), legend('H_\infty','LPV','location','best'), xlim([0 tlim])
end
end
end
exportgraphics(gcf,'figure/lat_ex_step_lin_p_input.png','resolution',300)
