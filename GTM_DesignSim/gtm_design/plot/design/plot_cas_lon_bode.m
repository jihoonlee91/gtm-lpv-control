w = {2*pi*1e-4,2*pi*1e4};

%% cas_lon_bode_an
figure('units','centimeters','position',[0 6 13 10]);

for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_lon_ex_hinf_point = CL_lon_ex_hinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CL_lon_ex_lpv_point = CL_lon_ex_lpv.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [mag,~,wout] = bode(CL_lon_ex_lpv_point(3,1),w);
    semilogx(wout/(2*pi),mag2db(squeeze(mag)/(W_r_an*Plonexscl(3,3))),'r')
    hold on
    [mag,~,wout] = bode(CL_lon_ex_hinf_point(3,1),w);
    semilogx(wout/(2*pi),mag2db(squeeze(mag)/(W_r_an*Plonexscl(3,3))),'--b')
    drawnow
end
end
end
legend('$LPV$','$H_\infty$','interpreter','latex','location','northeast')
xlabel('Frequency [Hz]')
ylabel('Magnitude [dB]')
grid on, grid minor
ylim padded

exportgraphics(gcf,'figure/cas_lon_bode_an.png','resolution',300)
