function plot_bode(CLlpvu,CLhinf,CLh2op,Weights,inum,onum,w)

set(0,'DefaultLineLineWidth',1)

figure('units','centimeters','position',[6 6 20 10]);

domain_size = size(CLlpvu.Domain);
num_sas = domain_size(1);
num_alt = domain_size(2);
num_mor = domain_size(3);

for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    CL_pt = CLlpvu.Data(:,:,i,j,k);
    [mag,~,wout] = bode(CL_pt(onum,inum),w);
    semilogx(wout/(2*pi),mag2db(squeeze(mag)/abs((Weights.Wr(inum,inum)*Weights.Ps(onum,onum)))),'b')
    hold on
    CL_pt = CLhinf.Data(:,:,i,j,k);
    [mag,~,wout] = bode(CL_pt(onum,inum),w);
    semilogx(wout/(2*pi),mag2db(squeeze(mag)/abs((Weights.Wr(inum,inum)*Weights.Ps(onum,onum)))),'--r')
    CL_pt = CLh2op.Data(:,:,i,j,k);
    [mag,~,wout] = bode(CL_pt(onum,inum),w);
    semilogx(wout/(2*pi),mag2db(squeeze(mag)/abs((Weights.Wr(inum,inum)*Weights.Ps(onum,onum)))),':g')
    drawnow
end
end
end
legend('LPV','H_\infty','H_2','orientation','horizontal','location','northeast')
xlabel('Frequency [Hz]')
ylabel('Magnitude [dB]')
grid on, grid minor
ylim padded
