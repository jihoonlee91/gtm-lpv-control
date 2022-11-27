%% bode
fig = figure('units','centimeters','position',[6.5 6 13 20]);
w = {1e-2,1e2};

subplot(2,1,1)
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    P_lon_point = P_lon.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [mag,~,wout] = bode(P_lon_point(1,1),w);
    semilogx(wout,mag2db(squeeze(mag*P_lon_scl(1,1))),'-.c')
    hold on, grid on, grid minor
    [mag,~,wout] = bode(P_lon_point(1,2),w);
    semilogx(wout,mag2db(squeeze(mag*P_lon_scl(1,1))),':m')
    title('')
    legend('Throttle [%]','Elevator [deg]','location','best')
    ylabel('True Airspeed [kt]')
    hold on, grid on, grid minor
end
end
end
grid minor

subplot(2,1,2)
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    P_lon_point = P_lon.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [mag,~,wout] = bode(P_lon_point(5,1),w);
    semilogx(wout,mag2db(squeeze(mag*K_lon_scl(1,1)*P_lon_scl(5,5))),'-.c')
    hold on, grid on, grid minor
    [mag,~,wout] = bode(P_lon_point(5,2),w);
    semilogx(wout,mag2db(squeeze(mag*K_lon_scl(2,2)*P_lon_scl(5,5))),':m')
    title('')
    legend('Throttle [%]','Elevator [deg]','location','best')
    ylabel('Altitude [ft]')
    hold on, grid on, grid minor
end
end
end
grid minor

exportgraphics(fig,'figure/open_loop_bode_lon.png','resolution',300)
% savefig(fig,'figure/open_loop_bode_lon.fig')

%% bode
fig = figure('units','centimeters','position',[6.5+13 6 13 20]);
w = {1e-2,1e2};

subplot(2,1,1)
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    P_lat_point = P_lat.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [mag,~,wout] = bode(P_lat_point(1,1),w);
    semilogx(wout,mag2db(squeeze(mag*P_lat_scl(1,1))),':c')
    hold on, grid on, grid minor
    [mag,~,wout] = bode(P_lat_point(1,2),w);
    semilogx(wout,mag2db(squeeze(mag*P_lat_scl(1,1))),':m')
    [mag,~,wout] = bode(P_lat_point(1,3),w);
    semilogx(wout,mag2db(squeeze(mag*P_lat_scl(1,1))),'-.g')
    hold on, grid on, grid minor
end
end
end
title('')
legend('Aileron [deg]','Rudder [deg]','Morphing [%]','location','best')
ylabel('Angle of Sideslip [deg]')
grid minor

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
    hold on, grid on, grid minor
end
end
end
title('')
legend('Aileron [deg]','Rudder [deg]','Morphing [%]','location','best')
ylabel('Roll Angle [deg]')
grid minor

exportgraphics(fig,'figure/open_loop_bode_lat.png','resolution',300)
% savefig(fig,'figure/open_loop_bode_lat.fig')
