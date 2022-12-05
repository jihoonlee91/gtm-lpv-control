function plot_bode(CLlpvu,CLhinf,Weights,inum,onum,w)

set(0,'DefaultLineLineWidth',1)

figure('units','centimeters','position',[6 6 20 14]);

domain_size = size(CLlpvu.Domain);
num_sas = domain_size(1);
num_alt = domain_size(2);
num_mor = domain_size(3);

for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
%     CLlpvu_pt = CLlpvu.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
%     CLhinf_pt = CLhinf.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    CLlpvu_pt = CLlpvu.Data(:,:,i,j,k);
    CLhinf_pt = CLhinf.Data(:,:,i,j,k);
    [mag,~,wout] = bode(CLlpvu_pt(onum,inum),w);
    semilogx(wout/(2*pi),mag2db(squeeze(mag)/abs((Weights.Wr(inum,inum)*Weights.Ps(onum,onum)))),'r')
    hold on
    [mag,~,wout] = bode(CLhinf_pt(onum,inum),w);
    semilogx(wout/(2*pi),mag2db(squeeze(mag)/abs((Weights.Wr(inum,inum)*Weights.Ps(onum,onum)))),'--b')
    drawnow
end
end
end
legend('$LPV$','$H_\infty$','interpreter','latex','location','northeast')
xlabel('Frequency [Hz]')
ylabel('Magnitude [dB]')
grid on, grid minor
ylim padded
