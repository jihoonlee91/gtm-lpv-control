set(0,'DefaultLineLineWidth',1)
tlim = 15;
%% tas step state response
fig1 = figure('units','centimeters','position',[0 6 13 20]);
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lon_lpv_point = CL_lon_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_hinf_point = CL_lon_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [y,t] = step(CL_lon_lpv_point,30);
    subplot(5,1,1), plot(t,y(:,1,1),'-','color',[0.8500 0.3250 0.0980]), ylabel('True Airspeed [kt]'), xlabel('Time [sec]')
    subplot(5,1,2), plot(t,y(:,2,1),'-','color',[0.8500 0.3250 0.0980]), ylabel('Angle of Attack [deg]'), xlabel('Time [sec]')
    subplot(5,1,3), plot(t,y(:,3,1),'-','color',[0.8500 0.3250 0.0980]), ylabel('Pitch Rate [deg/s]'), xlabel('Time [sec]')
    subplot(5,1,4), plot(t,y(:,4,1),'-','color',[0.8500 0.3250 0.0980]), ylabel('Pitch Angle [deg]'), xlabel('Time [sec]')
    subplot(5,1,5), plot(t,y(:,5,1),'-','color',[0.8500 0.3250 0.0980]), xlabel('Time [sec]'), ylabel('Altitude [ft]')
    if (i==1)&&(j==1)&&(k==1)
        subplot(5,1,1), hold on, grid on, grid minor
        subplot(5,1,2), hold on, grid on, grid minor 
        subplot(5,1,3), hold on, grid on, grid minor
        subplot(5,1,4), hold on, grid on, grid minor
        subplot(5,1,5), hold on, grid on, grid minor
    end
    [y,t] = step(CL_lon_hinf_point,30);
    subplot(5,1,1), plot(t,y(:,1,1),'--','color',[0 0.4470 0.7410]), ylabel('True Airspeed [kt]'), xlabel('Time [sec]')
    [y_r,t_r] = step(W_m_tas,tlim);
    plot(t_r,y_r*W_r_tas*P_lon_scl(1,1),'-.','color','k','linewidth',1.5), legend('LPV','H_\infty','Reference','location','best'), xlim([0 tlim])
    subplot(5,1,2), plot(t,y(:,2,1),'--','color',[0 0.4470 0.7410]), ylabel('Angle of Attack [deg]'), xlabel('Time [sec]'), legend('LPV','H_\infty','location','best'), xlim([0 tlim])
    subplot(5,1,3), plot(t,y(:,3,1),'--','color',[0 0.4470 0.7410]), ylabel('Pitch Rate [deg/s]'), xlabel('Time [sec]'), legend('LPV','H_\infty','location','best'), xlim([0 tlim])
    subplot(5,1,4), plot(t,y(:,4,1),'--','color',[0 0.4470 0.7410]), ylabel('Pitch Angle [deg]'), xlabel('Time [sec]'), legend('LPV','H_\infty','location','best'), xlim([0 tlim])
    subplot(5,1,5), plot(t,y(:,5,1),'--','color',[0 0.4470 0.7410]), xlabel('Time [sec]'), ylabel('Altitude [ft]')
    plot(t,y(:,1,1)*0,'-.','color','k','linewidth',1.5), legend('LPV','H_\infty','Reference','location','best'), xlim([0 tlim])
end
end
end
exportgraphics(fig1,'figure/lon_step_lin_tas_state.png','resolution',300)
savefig(fig1,'figure/lon_step_lin_tas_state.fig')

%% tas step input response
fig2 = figure('units','centimeters','position',[13 6 13 20]);
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lon_lpv_point = CL_lon_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_hinf_point = CL_lon_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [y,t] = step(CL_lon_lpv_point,30);
    subplot(2,1,1), plot(t,y(:,6,1),'-','color',[0.8500 0.3250 0.0980]), ylabel('Throttle [%]'), xlabel('Time [sec]')
    subplot(2,1,2), plot(t,y(:,7,1),'-','color',[0.8500 0.3250 0.0980]), xlabel('Time [sec]'), ylabel('Elevator [deg]')
    if (i==1)&&(j==1)&&(k==1)
        subplot(2,1,1), hold on, grid on, grid minor
        subplot(2,1,2), hold on, grid on, grid minor 
    end
    [y,t] = step(CL_lon_hinf_point,30);
    subplot(2,1,1), plot(t,y(:,6,1),'--','color',[0 0.4470 0.7410]), ylabel('Throttle [%]'), xlabel('Time [sec]'), legend('LPV','H_\infty','location','best'), xlim([0 tlim])
    subplot(2,1,2), plot(t,y(:,7,1),'--','color',[0 0.4470 0.7410]), xlabel('Time [sec]'), ylabel('Elevator [deg]'), legend('LPV','H_\infty','location','best'), xlim([0 tlim])
end
end
end
exportgraphics(fig2,'figure/lon_step_lin_tas_input.png','resolution',300)
savefig(fig2,'figure/lon_step_lin_tas_input.fig')

%% alt step state response
fig3 = figure('units','centimeters','position',[26 6 13 20]);
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lon_lpv_point = CL_lon_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_hinf_point = CL_lon_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [y,t] = step(CL_lon_lpv_point,30);
    subplot(5,1,1), plot(t,y(:,1,2),'-','color',[0.8500 0.3250 0.0980]), ylabel('True Airspeed [kt]'), xlabel('Time [sec]')
    subplot(5,1,2), plot(t,y(:,2,2),'-','color',[0.8500 0.3250 0.0980]), ylabel('Angle of Attack [deg]'), xlabel('Time [sec]')
    subplot(5,1,3), plot(t,y(:,3,2),'-','color',[0.8500 0.3250 0.0980]), ylabel('Pitch Rate [deg/s]'), xlabel('Time [sec]')
    subplot(5,1,4), plot(t,y(:,4,2),'-','color',[0.8500 0.3250 0.0980]), ylabel('Pitch Angle [deg]'), xlabel('Time [sec]')
    subplot(5,1,5), plot(t,y(:,5,2),'-','color',[0.8500 0.3250 0.0980]), xlabel('Time [sec]'), ylabel('Altitude [ft]')
    if (i==1)&&(j==1)&&(k==1)
        subplot(5,1,1), hold on, grid on, grid minor
        subplot(5,1,2), hold on, grid on, grid minor 
        subplot(5,1,3), hold on, grid on, grid minor
        subplot(5,1,4), hold on, grid on, grid minor
        subplot(5,1,5), hold on, grid on, grid minor
    end
    [y,t] = step(CL_lon_hinf_point,30);
    subplot(5,1,1), plot(t,y(:,1,2),'--','color',[0 0.4470 0.7410]), ylabel('True Airspeed [kt]'), xlabel('Time [sec]'), legend('LPV','H_\infty','location','best'), xlim([0 tlim])
    plot(t,y(:,1,1)*0+0,'-.','color','k','linewidth',1.5), legend('LPV','H_\infty','Reference','location','best'), xlim([0 tlim])
    subplot(5,1,2), plot(t,y(:,2,2),'--','color',[0 0.4470 0.7410]), ylabel('Angle of Attack [deg]'), xlabel('Time [sec]'), legend('LPV','H_\infty','location','best'), xlim([0 tlim])
    subplot(5,1,3), plot(t,y(:,3,2),'--','color',[0 0.4470 0.7410]), ylabel('Pitch Rate [deg/s]'), xlabel('Time [sec]'), legend('LPV','H_\infty','location','best'), xlim([0 tlim])
    subplot(5,1,4), plot(t,y(:,4,2),'--','color',[0 0.4470 0.7410]), ylabel('Pitch Angle [deg]'), xlabel('Time [sec]'), legend('LPV','H_\infty','location','best'), xlim([0 tlim])
    subplot(5,1,5), plot(t,y(:,5,2),'--','color',[0 0.4470 0.7410]), xlabel('Time [sec]'), ylabel('Altitude [ft]')
    [y_r,t_r] = step(W_m_alt,tlim);
    plot(t_r,y_r*W_r_alt*P_lon_scl(5,5),'-.','color','k','linewidth',1.5), legend('LPV','H_\infty','Reference','location','best'), xlim([0 tlim])
end
end
end
exportgraphics(fig3,'figure/lon_step_lin_alt_state.png','resolution',300)
savefig(fig3,'figure/lon_step_lin_alt_state.fig')

%% alt step input response
fig4 = figure('units','centimeters','position',[39 6 13 20]);
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lon_lpv_point = CL_lon_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_hinf_point = CL_lon_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [y,t] = step(CL_lon_lpv_point,30);
    subplot(2,1,1), plot(t,y(:,6,2),'-','color',[0.8500 0.3250 0.0980]), ylabel('Throttle [%]'), xlabel('Time [sec]')
    subplot(2,1,2), plot(t,y(:,7,2),'-','color',[0.8500 0.3250 0.0980]), xlabel('Time [sec]'), ylabel('Elevator [deg]')
    if (i==1)&&(j==1)&&(k==1)
        subplot(2,1,1), hold on, grid on, grid minor
        subplot(2,1,2), hold on, grid on, grid minor 
    end
    [y,t] = step(CL_lon_hinf_point,30);
    subplot(2,1,1), plot(t,y(:,6,2),'--','color',[0 0.4470 0.7410]), ylabel('Throttle [%]'), xlabel('Time [sec]'), legend('LPV','H_\infty','location','best'), xlim([0 tlim])
    subplot(2,1,2), plot(t,y(:,7,2),'--','color',[0 0.4470 0.7410]), xlabel('Time [sec]'), ylabel('Elevator [deg]'), legend('LPV','H_\infty','location','best'), xlim([0 tlim])
end
end
end
exportgraphics(fig4,'figure/lon_step_lin_alt_input.png','resolution',300)
savefig(fig4,'figure/lon_step_lin_alt_input.fig')
