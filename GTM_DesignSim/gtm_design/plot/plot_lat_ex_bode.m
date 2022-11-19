%% bode
fig = figure('units','centimeters','position',[6.5 2 13 20]);
w = {1e-2,1e2};

subplot(2,1,1)
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    P_lat_ex_point = P_lat_ex.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_ex_lpv_point = CL_lat_ex_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_ex_hinf_point = CL_lat_ex_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    bodemag(CL_lat_ex_lpv_point(1,1)/(W_r_bet*Platexscl(1,1)),w,'r')
    hold on, grid on, grid minor
    bodemag(CL_lat_ex_hinf_point(1,1)/(W_r_bet*Platexscl(1,1)),w,'--b')
    [mag,~,wout] = bode(P_lat_ex_point(1,1),w);
    semilogx(wout,mag2db(squeeze(mag*P_lat_scl(1,1))),':c')
    [mag,~,wout] = bode(P_lat_ex_point(1,2),w);
    semilogx(wout,mag2db(squeeze(mag*P_lat_scl(1,1))),':m')
    [mag,~,wout] = bode(P_lat_ex_point(1,3),w);
    semilogx(wout,mag2db(squeeze(mag*P_lat_scl(1,1))),'-.g')
end
end
end
title('')
legend('LPV','H-Infinity','Aileron [deg]','Rudder [deg]','Morphing [%]','location','best')
ylabel('Angle of Sideslip [deg]')

subplot(2,1,2)
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    P_lat_ex_point = P_lat_ex.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_ex_lpv_point = CL_lat_ex_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_ex_hinf_point = CL_lat_ex_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    bodemag(CL_lat_ex_lpv_point(2,2)/45,w,'r')
    hold on, grid on, grid minor
    bodemag(CL_lat_ex_hinf_point(2,2)/45,w,'--b')
    [mag,~,wout] = bode(P_lat_ex_point(2,1),w);
    semilogx(wout,mag2db(squeeze(mag)*r2d),':c')
    [mag,~,wout] = bode(P_lat_ex_point(2,3),w);
    semilogx(wout,mag2db(squeeze(mag)*r2d),':m')
    [mag,~,wout] = bode(P_lat_ex_point(2,3),w);
    semilogx(wout,mag2db(squeeze(mag)*r2d),'-.g')
end
end
end
title('')
legend('LPV','H-Infinity','Aileron [deg]','Rudder [deg]','Morphing [%]','location','best')
ylabel('Roll Rate [deg/s]')

exportgraphics(fig,'figure/lat_ex_bode.png','resolution',300)
savefig(fig,'figure/lat_ex_bode.fig')
