close all
clear
clc

spec_data
aero_data

set(groot,'DefaultAxesFontSize',10)
set(groot,'DefaultLineLineWidth',2)

figure('position',[0 100 1000 800])
for ii = 1:3
    for jj = 1:3
        subplot(3,3,3*(ii-1)+jj)
        f = fit(rad2deg(alp_grd.'),CL_grd(:,2,ii,jj),'poly1');
        plot(f,rad2deg(alp_grd.'),CL_grd(:,2,ii,jj))
        title(['C_L=',num2str(f.p2),'+',num2str(f.p1),'\alpha',' (\eta=[',num2str(eta1_grd(ii)),' ',num2str(eta2_grd(jj)),']^T)'])
        legend('off')
        hold on, grid on
        xlim([-10 20])
        ylim([min(min(min(CL_grd(:,2,:,:)))) max(max(max(CL_grd(:,2,:,:))))])
        xlabel(['\alpha (',sprintf('%c', char(176)),')'])
        ylabel('C_L')
        
        CL_0(ii,jj) = f.p2;
        CL_a(ii,jj) = f.p1;
    end
end

figure('position',[100 100 1000 800])
for ii = 1:3
    for jj = 1:3
        subplot(3,3,3*(ii-1)+jj)
        f = fit(rad2deg(alp_grd.'),CD_grd(:,2,ii,jj),'poly2');
        plot(f,rad2deg(alp_grd.'),CD_grd(:,2,ii,jj))
        title(['C_D=',num2str(f.p3),'+',num2str(f.p2),'\alpha','+',num2str(f.p1),'\alpha^2',' (\eta=[',num2str(eta1_grd(ii)),' ',num2str(eta2_grd(jj)),']^T)'])
        legend('off')
        hold on, grid on
        xlim([-10 20])
        xlim([-10 20])
        ylim([min(min(min(CD_grd(:,2,:,:)))) max(max(max(CD_grd(:,2,:,:))))])
        xlabel(['\alpha (',sprintf('%c', char(176)),')'])
        ylabel('C_D')

        CD_0(ii,jj) = f.p3;
        CD_a(ii,jj) = f.p2;
        CD_a2(ii,jj) = f.p1;
    end
end

figure('position',[200 100 1000 800])
for ii = 1:3
    for jj = 1:3
        subplot(3,3,3*(ii-1)+jj)
        f = fit(rad2deg(alp_grd.'),Cm_grd(:,2,ii,jj),'poly1');
        plot(f,rad2deg(alp_grd.'),Cm_grd(:,2,ii,jj))
        title(['C_m=',num2str(f.p2),'+',num2str(f.p1),'\alpha',' (\eta=[',num2str(eta1_grd(ii)),' ',num2str(eta2_grd(jj)),']^T)'])
        legend('off')
        hold on, grid on
        xlim([-10 20])
        xlim([-10 20]), ylim([min(min(min(Cm_grd(:,2,:,:)))) max(max(max(Cm_grd(:,2,:,:))))])
        xlabel(['\alpha (',sprintf('%c', char(176)),')'])
        ylabel('C_m')
        ylim([-4 2])
        
        Cm_0(ii,jj) = f.p2;
        Cm_a(ii,jj) = f.p1;
    end
end

figure('position',[300 100 1000 800])
for ii = 1:3
    for jj = 1:3
        subplot(3,3,3*(ii-1)+jj)
        f = fit(rad2deg(dele_grd.'),Cm_grd(21,:,ii,jj).'-Cm_0(ii,jj),'poly1');
        plot(f,rad2deg(dele_grd.'),Cm_grd(21,:,ii,jj)-Cm_0(ii,jj).')
        title(['C_{m_{\delta_e}}=',num2str(f.p1),' (\eta=[',num2str(eta1_grd(ii)),' ',num2str(eta2_grd(jj)),']^T)'])
        legend('off')
        hold on, grid on
        xlim([-10 10]), ylim([-2 2])
        xlabel(['\delta_e (',sprintf('%c', char(176)),')'])
        ylabel('C_m(\delta_e)')
        Cm_de(ii,jj) = f.p1;
    end
end

figure('position',[400 100 1000 800])
for ii = 1:3
    for jj = 1:3
        L2D(:,1,ii,jj) = CL_grd(:,2,ii,jj)./CD_grd(:,2,ii,jj);
        
        subplot(3,3,3*(ii-1)+jj)
        plot(rad2deg(alp_grd.'),L2D(:,1,ii,jj))
        legend('off')
        hold on, grid on
        xlim([-10 20])
        xlabel(['\alpha (',sprintf('%c', char(176)),')'])
        ylabel('L/D')
        ylim([min(min(min(CL_grd(:,2,:,:)./CD_grd(:,2,:,:)))) max(max(max(CL_grd(:,2,:,:)./CD_grd(:,2,:,:))))])
    end
end

figure('position',[400 100 500 400])
for ii = 1:3
for jj = 1:3
plot(CD_grd(:,2,ii,jj),CL_grd(:,2,ii,jj))
hold on
xlabel('C_D')
ylabel('C_L')
% xlim([0 0.16])
% ylim([-0.6 1.4])
grid on
title('Drag Polar')
end
end
legend('show')
legend('location','best')

%%

figure('position',[300 100 1000 800])
for ii = 1:3
    for jj = 1:3
        subplot(3,3,3*(ii-1)+jj)
        f = fit(rad2deg(dele_grd.'),CL_grd(21,:,ii,jj).'-CL_0(ii,jj),'poly1');
        plot(f,rad2deg(dele_grd.'),CL_grd(21,:,ii,jj)-CL_0(ii,jj).')
        title(['C_{L_{\delta_e}}=',num2str(f.p1),' (\eta=[',num2str(eta1_grd(ii)),' ',num2str(eta2_grd(jj)),']^T)'])
        legend('off')
        hold on, grid on
        xlim([-10 10]), ylim([-2 2])
        xlabel(['\delta_e (',sprintf('%c', char(176)),')'])
        ylabel('C_L(\delta_e)')
        CL_de(ii,jj) = f.p1;
    end
end

figure('position',[300 100 1000 800])
for ii = 1:3
    for jj = 1:3
        subplot(3,3,3*(ii-1)+jj)
        f = fit(rad2deg(dele_grd.'),CD_grd(21,:,ii,jj).'-CD_0(ii,jj),'poly1');
        plot(f,rad2deg(dele_grd.'),CD_grd(21,:,ii,jj)-CD_0(ii,jj).')
        title(['C_{D_{\delta_e}}=',num2str(f.p1),' (\eta=[',num2str(eta1_grd(ii)),' ',num2str(eta2_grd(jj)),']^T)'])
        legend('off')
        hold on, grid on
        xlim([-10 10]), ylim([-2 2])
        xlabel(['\delta_e (',sprintf('%c', char(176)),')'])
        ylabel('C_D(\delta_e)')
        CD_de(ii,jj) = f.p1;
    end
end
