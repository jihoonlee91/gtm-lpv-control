tlim = 4;

%% bet step state response
fig1 = figure('units','centimeters','position',[0 2 13 20]);
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lat_lpv_point = CL_lat_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_hinf_point = CL_lat_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [y,t] = step(CL_lat_lpv_point,tlim);
    subplot(4,1,1), plot(t,y(:,1,1),'color',[0.8500 0.3250 0.0980]), ylabel('Angle of Sideslip [deg]'), xlabel('Time [sec]')
    subplot(4,1,2), plot(t,y(:,2,1),'color',[0.8500 0.3250 0.0980]), ylabel('Roll Rate [deg/s]'), xlabel('Time [sec]')
    subplot(4,1,3), plot(t,y(:,3,1),'color',[0.8500 0.3250 0.0980]), ylabel('Yaw Rate [deg/s]'), xlabel('Time [sec]')
    subplot(4,1,4), plot(t,y(:,4,1),'color',[0.8500 0.3250 0.0980]), xlabel('Time [sec]'), ylabel('Roll Angle [deg]')
    if (i==1)&&(j==1)&&(k==1)
        subplot(4,1,1), hold on, grid on, grid minor
        subplot(4,1,2), hold on, grid on, grid minor 
        subplot(4,1,3), hold on, grid on, grid minor
        subplot(4,1,4),  hold on, grid on, grid minor
    end
    [y,t] = step(CL_lat_hinf_point,tlim);
    subplot(4,1,1), plot(t,y(:,1,1),'--','color',[0 0.4470 0.7410]), ylabel('Angle of Sideslip [deg]'), xlabel('Time [sec]')
    [y_r,t_r] = step(W_m_bet,tlim);
    plot(t_r,y_r*W_r_bet*P_lat_scl(1,1),'-.','color','k','linewidth',1.5), legend('LPV','H_\infty','Reference','location','best')
    subplot(4,1,2), plot(t,y(:,2,1),'--','color',[0 0.4470 0.7410]), ylabel('Roll Rate [deg/s]'), xlabel('Time [sec]'), legend('LPV','H_\infty','location','best')
    subplot(4,1,3), plot(t,y(:,3,1),'--','color',[0 0.4470 0.7410]), ylabel('Yaw Rate [deg/s]'), xlabel('Time [sec]'), legend('LPV','H_\infty','location','best')
    subplot(4,1,4), plot(t,y(:,4,1),'--','color',[0 0.4470 0.7410]), xlabel('Time [sec]'), ylabel('Roll Angle [deg]')
    plot(t,y(:,1,1)*0+0,'-.','color','k','linewidth',1.5), legend('LPV','H_\infty','Reference','location','best'), xlim([0 tlim])
end
end
end
exportgraphics(fig1,'figure/lat_step_lin_bet_state.png','resolution',300)
% savefig(fig1,'figure/lat_step_lin_bet_state.fig')

%% bet step input response
fig2 = figure('units','centimeters','position',[13 2 13 20]);
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lat_lpv_point = CL_lat_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_hinf_point = CL_lat_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [y,t] = step(CL_lat_lpv_point,tlim);
    subplot(3,1,1), plot(t,y(:,5,1),'color',[0.8500 0.3250 0.0980]), ylabel('Aileron [deg]'), xlabel('Time [sec]')
    subplot(3,1,2), plot(t,y(:,6,1),'color',[0.8500 0.3250 0.0980]), ylabel('Rudder [deg]'), xlabel('Time [sec]')
    subplot(3,1,3), plot(t,y(:,7,1),'color',[0.8500 0.3250 0.0980]), xlabel('Time [sec]'), ylabel('Asymmetric Morphing [%]')
    if (i==1)&&(j==1)&&(k==1)
        subplot(3,1,1), hold on, grid on, grid minor
        subplot(3,1,2), hold on, grid on, grid minor 
        subplot(3,1,3), hold on, grid on, grid minor 
    end
    [y,t] = step(CL_lat_hinf_point,tlim);
    subplot(3,1,1), plot(t,y(:,5,1),'--','color',[0 0.4470 0.7410]), ylabel('Aileron [deg]'), xlabel('Time [sec]'), legend('LPV','H_\infty','location','best'), xlim([0 tlim])
    subplot(3,1,2), plot(t,y(:,6,1),'--','color',[0 0.4470 0.7410]), ylabel('Rudder [deg]'), xlabel('Time [sec]'), legend('LPV','H_\infty','location','best'), xlim([0 tlim])
    subplot(3,1,3), plot(t,y(:,7,1),'--','color',[0 0.4470 0.7410]), xlabel('Time [sec]'), ylabel('Asymmetric Morphing [%]'), legend('LPV','H_\infty','location','best'), xlim([0 tlim])
end
end
end
exportgraphics(fig2,'figure/lat_step_lin_bet_input.png','resolution',300)
% savefig(fig2,'figure/lat_step_lin_bet_input.fig')

%% phi step state response
fig3 = figure('units','centimeters','position',[26 2 13 20]);
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lat_lpv_point = CL_lat_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_hinf_point = CL_lat_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [y,t] = step(CL_lat_lpv_point,tlim);
    subplot(4,1,1), plot(t,y(:,1,2),'color',[0.8500 0.3250 0.0980]), ylabel('Angle of Sideslip [deg]'), xlabel('Time [sec]')
    subplot(4,1,2), plot(t,y(:,2,2),'color',[0.8500 0.3250 0.0980]), ylabel('Roll Rate [deg/s]'), xlabel('Time [sec]')
    subplot(4,1,3), plot(t,y(:,3,2),'color',[0.8500 0.3250 0.0980]), ylabel('Yaw Rate [deg/s]'), xlabel('Time [sec]')
    subplot(4,1,4), plot(t,y(:,4,2),'color',[0.8500 0.3250 0.0980]), xlabel('Time [sec]'), ylabel('Roll Angle [deg]')
    if (i==1)&&(j==1)&&(k==1)
        subplot(4,1,1), hold on, grid on, grid minor
        subplot(4,1,2), hold on, grid on, grid minor 
        subplot(4,1,3), hold on, grid on, grid minor
        subplot(4,1,4), hold on, grid on, grid minor
    end
    [y,t] = step(CL_lat_hinf_point,tlim);
    subplot(4,1,1), plot(t,y(:,1,2),'--','color',[0 0.4470 0.7410]), ylabel('Angle of Sideslip [deg]'), xlabel('Time [sec]')
    plot(t,y(:,1,1)*0+0,'-.','color','k','linewidth',1.5), legend('LPV','H_\infty','Reference','location','best')
    subplot(4,1,2), plot(t,y(:,2,2),'--','color',[0 0.4470 0.7410]), ylabel('Roll Rate [deg/s]'), xlabel('Time [sec]'), legend('LPV','H_\infty','location','best')
    subplot(4,1,3), plot(t,y(:,3,2),'--','color',[0 0.4470 0.7410]), ylabel('Yaw Rate [deg/s]'), xlabel('Time [sec]'), legend('LPV','H_\infty','location','best')
    subplot(4,1,4), plot(t,y(:,4,2),'--','color',[0 0.4470 0.7410]), xlabel('Time [sec]'), ylabel('Roll Angle [deg]')
    [y_r,t_r] = step(W_m_phi,tlim);
    plot(t_r,y_r*W_r_phi*P_lat_scl(4,4),'-.','color','k','linewidth',1.5), legend('LPV','H_\infty','Reference','location','best'), xlim([0 tlim])
end
end
end
exportgraphics(fig3,'figure/lat_step_lin_phi_state.png','resolution',300)
% savefig(fig3,'figure/lat_step_lin_phi_state.fig')

%% phi step input response
fig4 = figure('units','centimeters','position',[39 2 13 20]);
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lat_lpv_point = CL_lat_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_hinf_point = CL_lat_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [y,t] = step(CL_lat_lpv_point,tlim);
    subplot(3,1,1), plot(t,y(:,5,2),'color',[0.8500 0.3250 0.0980]), ylabel('Aileron [deg]'), xlabel('Time [sec]')
    subplot(3,1,2), plot(t,y(:,6,2),'color',[0.8500 0.3250 0.0980]), ylabel('Rudder [deg]'), xlabel('Time [sec]')
    subplot(3,1,3), plot(t,y(:,7,2),'color',[0.8500 0.3250 0.0980]), xlabel('Time [sec]'), ylabel('Asymmetric Morphing [%]')
    if (i==1)&&(j==1)&&(k==1)
        subplot(3,1,1), hold on, grid on, grid minor
        subplot(3,1,2), hold on, grid on, grid minor 
        subplot(3,1,3), hold on, grid on, grid minor 
    end
    [y,t] = step(CL_lat_hinf_point,tlim);
    subplot(3,1,1), plot(t,y(:,5,2),'--','color',[0 0.4470 0.7410]), ylabel('Aileron [deg]'), xlabel('Time [sec]'), legend('LPV','H_\infty','location','best'), xlim([0 tlim])
    subplot(3,1,2), plot(t,y(:,6,2),'--','color',[0 0.4470 0.7410]), ylabel('Rudder [deg]'), xlabel('Time [sec]'), legend('LPV','H_\infty','location','best'), xlim([0 tlim])
    subplot(3,1,3), plot(t,y(:,7,2),'--','color',[0 0.4470 0.7410]), xlabel('Time [sec]'), ylabel('Asymmetric Morphing [%]'), legend('LPV','H_\infty','location','best'), xlim([0 tlim])
end
end
end
exportgraphics(fig4,'figure/lat_step_lin_phi_input.png','resolution',300)
% savefig(fig4,'figure/lat_step_lin_phi_input.fig')