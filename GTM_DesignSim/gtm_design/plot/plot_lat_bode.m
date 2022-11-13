%% bode
figure('units','centimeters','position',[6.5 2 13 20])
w = {1e-2,1e2};

subplot(2,1,1)
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    P_lat_point = P_lat.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_hinf_point = CL_lat_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_lpv_point = CL_lat_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [mag,~,wout] = bode(P_lat_point(1,2),w);
    semilogx(wout,mag2db(squeeze(mag)*r2d),'-.k')
    hold on, grid on, grid minor
    [mag,~,wout] = bode(P_lat_point(1,3),w);
    semilogx(wout,mag2db(squeeze(mag)*r2d),':g')
    bodemag(CL_lat_hinf_point(1,1)/3,w,'b')
    bodemag(CL_lat_lpv_point(1,1)/3,w,'--r')
end
end
end
title('')
legend('Rudder [deg]','Morphing [%]','H-Infinity','LPV','location','best')
ylabel('Angle of Sideslip [deg]')

subplot(2,1,2)
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    P_lat_point = P_lat.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_hinf_point = CL_lat_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_lpv_point = CL_lat_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [mag,~,wout] = bode(P_lat_point(4,1),w);
    semilogx(wout,mag2db(squeeze(mag)*r2d),'-.k')
    hold on, grid on, grid minor
    [mag,~,wout] = bode(P_lat_point(4,3),w);
    semilogx(wout,mag2db(squeeze(mag)*r2d),':g')
    bodemag(CL_lat_hinf_point(4,2)/45,w,'b')
    bodemag(CL_lat_lpv_point(4,2)/45,w,'--r')
end
end
end
title('')
legend('Aileron [deg]','Morphing [%]','H-Infinity','LPV','location','best')
ylabel('Roll Angle [deg]')

exportgraphics(gcf,'figure/lat_bode.png','resolution',300)
