w = {2*pi*1e-4,2*pi*1e4};

%% auto_lat_bode_bet
figure('units','centimeters','position',[0 6 13 10]);

for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lat_hinf_point = CL_lat_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_lpv_point = CL_lat_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [mag,~,wout] = bode(CL_lat_lpv_point(1,1),w);
    semilogx(wout/(2*pi),mag2db(squeeze(mag)/(W_r_bet*P_lat_scl(1,1))),'r')
    hold on
    [mag,~,wout] = bode(CL_lat_hinf_point(1,1),w);
    semilogx(wout/(2*pi),mag2db(squeeze(mag)/(W_r_bet*P_lat_scl(1,1))),'--b')
    drawnow
end
end
end
legend('$LPV$','$H_\infty$','interpreter','latex','location','northeast')
xlabel('Frequency [Hz]')
ylabel('Magnitude [dB]')
grid on, grid minor
ylim padded

exportgraphics(gcf,'figure/auto_lat_bode_bet.png','resolution',300)

%% auto_lat_bode_phi
figure('units','centimeters','position',[0+13 6 13 10]);

for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lat_hinf_point = CL_lat_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lat_lpv_point = CL_lat_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [mag,~,wout] = bode(CL_lat_lpv_point(4,2),w);
    semilogx(wout/(2*pi),mag2db(squeeze(mag)/(W_r_phi*P_lat_scl(4,4))),'r')
    hold on
    [mag,~,wout] = bode(CL_lat_hinf_point(4,2),w);
    semilogx(wout/(2*pi),mag2db(squeeze(mag)/(W_r_phi*P_lat_scl(4,4))),'--b')
    drawnow
end
end
end
legend('$LPV$','$H_\infty$','interpreter','latex','location','northeast')
xlabel('Frequency [Hz]')
ylabel('Magnitude [dB]')
grid on, grid minor
ylim padded

exportgraphics(gcf,'figure/auto_lat_bode_phi.png','resolution',300)
