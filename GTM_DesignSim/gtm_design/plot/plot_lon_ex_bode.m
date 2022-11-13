%% bode
figure('units','centimeters','position',[6.5 7 13 10])
w = {1e-2,1e2};

for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    P_lon_ex_point = P_lon_ex.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_ex_hinf_point = CL_lon_ex_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_ex_lpv_point = CL_lon_ex_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [mag,~,wout] = bode(P_lon_ex_point(3,1)*fps2knots,w);
    semilogx(wout,mag2db(squeeze(mag)/g),'-.k')
    hold on, grid on, grid minor
    bodemag(CL_lon_ex_hinf_point(3,1)/3,w,'b')
    bodemag(CL_lon_ex_lpv_point(3,1)/3,w,'--r')
end
end
end
title('')
legend('Elevator [deg]','H-Infinity','LPV','location','best')
ylabel('Normal Acceleration [g]')

exportgraphics(gcf,'figure/lon_ex_bode.png','resolution',300)
