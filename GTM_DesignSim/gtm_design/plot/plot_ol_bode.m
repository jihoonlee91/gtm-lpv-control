%% bode
fig = figure('units','centimeters','position',[6.5 6 13 20]);
w = {1e-2,1e2};

subplot(2,1,1)
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    P_lon_point = P_lon.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [mag,~,wout] = bode(P_lon_point(1,1),w);
    semilogx(wout,mag2db(squeeze(mag*fps2knots)),'-.c')
    hold on, grid on, grid minor
    [mag,~,wout] = bode(P_lon_point(1,2),w);
    semilogx(wout,mag2db(squeeze(mag*fps2knots)),':m')
    title('')
    legend('Throttle [%]','Elevator [deg]','location','best')
    ylabel('True Airspeed [kt]')
end
end
end

subplot(2,1,2)
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    P_lon_point = P_lon.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [mag,~,wout] = bode(P_lon_point(5,1),w);
    semilogx(wout,mag2db(squeeze(mag)),'-.c')
    hold on, grid on, grid minor
    [mag,~,wout] = bode(P_lon_point(5,2),w);
    semilogx(wout,mag2db(squeeze(mag)),':m')
    title('')
    legend('Throttle [%]','Elevator [deg]','location','best')
    ylabel('Altitude [ft]')
end
end
end

exportgraphics(fig,'figure/lon_bode.png','resolution',300)
% savefig(fig,'figure/lon_bode.fig')

%% bode
fig = figure('units','centimeters','position',[6.5 2 13 20]);
w = {1e-2,1e2};

subplot(2,1,1)
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    P_lat_point = P_lat.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [mag,~,wout] = bode(P_lat_point(1,1),w);
    semilogx(wout,mag2db(squeeze(mag*r2d)),':c')
    hold on, grid on, grid minor
    [mag,~,wout] = bode(P_lat_point(1,2),w);
    semilogx(wout,mag2db(squeeze(mag*r2d)),':m')
    [mag,~,wout] = bode(P_lat_point(1,3),w);
    semilogx(wout,mag2db(squeeze(mag*r2d)),'-.g')
end
end
end
title('')
legend('Aileron [deg]','Rudder [deg]','Morphing [%]','location','best')
ylabel('Angle of Sideslip [deg]')

subplot(2,1,2)
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    P_lat_point = P_lat.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [mag,~,wout] = bode(P_lat_point(4,1),w);
    semilogx(wout,mag2db(squeeze(mag)*r2d),':c')
    hold on, grid on, grid minor
    [mag,~,wout] = bode(P_lat_point(4,2),w);
    semilogx(wout,mag2db(squeeze(mag)*r2d),':m')
    [mag,~,wout] = bode(P_lat_point(4,3),w);
    semilogx(wout,mag2db(squeeze(mag)*r2d),'-.g')
end
end
end
title('')
legend('Aileron [deg]','Rudder [deg]','Morphing [%]','location','best')
ylabel('Roll Angle [deg]')

exportgraphics(fig,'figure/lat_bode.png','resolution',300)
% savefig(fig,'figure/lat_bode.fig')

clearvars fig