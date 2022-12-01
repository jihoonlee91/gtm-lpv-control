w = {2*pi*1e-4,2*pi*1e4};

%% auto_lon_bode_tas
figure('units','centimeters','position',[0 6 13 10]);

for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lon_hinf_point = CL_lon_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_lpv_point = CL_lon_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [mag,~,wout] = bode(CL_lon_lpv_point(1,1),w);
    semilogx(wout/(2*pi),mag2db(squeeze(mag)/(W_r_tas*P_lon_scl(1,1))),'r')
    hold on
    [mag,~,wout] = bode(CL_lon_hinf_point(1,1),w);
    semilogx(wout/(2*pi),mag2db(squeeze(mag)/(W_r_tas*P_lon_scl(1,1))),'--b')
    drawnow
end
end
end
legend('$LPV$','$H_\infty$','interpreter','latex','location','northeast')
xlabel('Frequency [Hz]')
ylabel('Magnitude [dB]')
grid on, grid minor
ylim padded

exportgraphics(gcf,'figure/auto_lon_bode_tas.png','resolution',300)

%% auto_lon_bode_alt
figure('units','centimeters','position',[0+13 6 13 10]);

for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lon_hinf_point = CL_lon_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_lpv_point = CL_lon_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [mag,~,wout] = bode(CL_lon_lpv_point(5,2),w);
    semilogx(wout/(2*pi),mag2db(squeeze(mag)/(W_r_alt*P_lon_scl(5,5))),'r')
    hold on
    [mag,~,wout] = bode(CL_lon_hinf_point(5,2),w);
    semilogx(wout/(2*pi),mag2db(squeeze(mag)/(W_r_alt*P_lon_scl(5,5))),'--b')
    drawnow
end
end
end
legend('$LPV$','$H_\infty$','interpreter','latex','location','northeast')
xlabel('Frequency [Hz]')
ylabel('Magnitude [dB]')
grid on, grid minor
ylim padded

exportgraphics(gcf,'figure/auto_lon_bode_alt.png','resolution',300)
