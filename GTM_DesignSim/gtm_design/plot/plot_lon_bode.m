%% bode
fig = figure('units','centimeters','position',[6.5 7 13 20]);
w = {1e-2,1e2};

subplot(2,1,1)
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    P_lon_point = P_lon.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_hinf_point = CL_lon_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_lpv_point = CL_lon_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [mag,~,wout] = bode(P_lon_point(1,1)*fps2knots,w);
    semilogx(wout,mag2db(squeeze(mag)),'-.k')
    hold on, grid on, grid minor
    bodemag(CL_lon_hinf_point(1,1)/10,w,'b')
    bodemag(CL_lon_lpv_point(1,1)/10,w,'--r')
end
end
end
title('')
legend('Throttle [%]','H-Infinity','LPV','location','best')
ylabel('True Airspeed [kt]')

subplot(2,1,2)
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    P_lon_point = P_lon.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_hinf_point = CL_lon_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_lpv_point = CL_lon_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [mag,~,wout] = bode(P_lon_point(5,2),w);
    semilogx(wout,mag2db(squeeze(mag)),'-.k')
    hold on, grid on, grid minor
    bodemag(CL_lon_hinf_point(5,2)/10,w,'b')
    bodemag(CL_lon_lpv_point(5,2)/10,w,'--r')
end
end
end
title('')
legend('Elevator [deg]','H-Infinity','LPV','location','best')
ylabel('Altitude [ft]')

exportgraphics(fig,'figure/lon_bode.png','resolution',300)
savefig(fig,'figure/lon_bode.fig')
