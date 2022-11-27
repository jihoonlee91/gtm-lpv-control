%% bode
fig = figure('units','centimeters','position',[6.5 6 13 10]);
w = {1e-2,1e2};

for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    P_lon_ex_point = P_lon_ex.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_ex_lpv_point = CL_lon_ex_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_ex_hinf_point = CL_lon_ex_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    bodemag(CL_lon_ex_lpv_point(3,1)/(W_r_an*Plonexscl(3,3)),w,'r')
    hold on, grid on, grid minor
    bodemag(CL_lon_ex_hinf_point(3,1)/(W_r_an*Plonexscl(3,3)),w,'--b')
    [mag,~,wout] = bode(P_lon_ex_point(3,1)*Plonexscl(3,3),w);
    semilogx(wout,mag2db(squeeze(mag)/g),'-.m')
end
end
end
title('')
legend('LPV','H-Infinity','Elevator [deg]','location','best')
ylabel('Normal Acceleration [g]')

exportgraphics(fig,'figure/lon_ex_bode.png','resolution',300)
% savefig(fig,'figure/lon_ex_bode.fig')

clearvars fig