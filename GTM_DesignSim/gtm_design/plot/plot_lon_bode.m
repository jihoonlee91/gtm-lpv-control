%% bode
fig = figure('units','centimeters','position',[6.5 6 13 20]);
w = {1e-2,1e2};

subplot(2,1,1)
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    P_lon_point = P_lon.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_hinf_point = CL_lon_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_lpv_point = CL_lon_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    bodemag(CL_lon_lpv_point(1,1)/(W_r_tas*P_lon_scl(1,1)),w,'r')
    hold on, grid on, grid minor
    bodemag(CL_lon_hinf_point(1,1)/(W_r_tas*P_lon_scl(1,1)),w,'--b')
    [mag,~,wout] = bode(P_lon_point(1,1),w);
    semilogx(wout,mag2db(squeeze(mag*P_lon_scl(1,1))),'-.c')
    [mag,~,wout] = bode(P_lon_point(1,2),w);
    semilogx(wout,mag2db(squeeze(mag*P_lon_scl(1,1))),':m')
    title('')
    legend('LPV','H-Infinity','Throttle [%]','Elevator [deg]','location','best')
    ylabel('True Airspeed [kt]')
end
end
end

subplot(2,1,2)
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    P_lon_point = P_lon.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_hinf_point = CL_lon_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_lpv_point = CL_lon_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    bodemag(CL_lon_lpv_point(5,2)/(W_r_alt*P_lon_scl(5,5)),w,'--r')
    hold on, grid on, grid minor
    bodemag(CL_lon_hinf_point(5,2)/(W_r_alt*P_lon_scl(5,5)),w,'b')
    [mag,~,wout] = bode(P_lon_point(5,1),w);
    semilogx(wout,mag2db(squeeze(mag*K_lon_scl(1,1)*P_lon_scl(5,5))),'-.c')
    [mag,~,wout] = bode(P_lon_point(5,2),w);
    semilogx(wout,mag2db(squeeze(mag*K_lon_scl(2,2)*P_lon_scl(5,5))),':m')
    title('')
    legend('LPV','H-Infinity','Throttle [%]','Elevator [deg]','location','best')
    ylabel('Altitude [ft]')
end
end
end

exportgraphics(fig,'figure/lon_bode.png','resolution',300)
% savefig(fig,'figure/lon_bode.fig')
