set(0,'DefaultLineLineWidth',1)
% figure('units','centimeters','position',[0 2 24 15]);
% for j = 1:num_alt
% for i = 1:num_sas
%     subplot(1,3,j)
%     sas = sas_grid(i); % trim sas [kt]
%     alt = alt_grid(j); % trim alt [ft]
%     tas = sas2tas(sas,alt); % [kts]
%     yyaxis left
%     plot(tas*ones(num_mor,1),squeeze(sch_param.sch_alp_trim.data.D(:,:,i,j,:)),'-'), hold on
%     scatter(tas*ones(num_mor,1),squeeze(sch_param.sch_alp_trim.data.D(:,:,i,j,1)),'x')
%     scatter(tas*ones(num_mor,1),squeeze(sch_param.sch_alp_trim.data.D(:,:,i,j,3)),'o')
%     ylabel('Angle of Attack [deg]')
%     ylim padded
%     yyaxis right
%     plot(tas*ones(num_mor,1),squeeze(sch_param.sch_throttle_trim.data.D(:,:,i,j,:)),'--'), hold on
%     scatter(tas*ones(num_mor,1),squeeze(sch_param.sch_throttle_trim.data.D(:,:,i,j,1)),'x')
%     scatter(tas*ones(num_mor,1),squeeze(sch_param.sch_throttle_trim.data.D(:,:,i,j,3)),'o')
%     ylabel('Throttle [%]')
%     ylim padded
%     grid on, grid minor
%     xlabel('True Airspeed [kt]')
%     title(['h = ',num2str(alt),' ft'],'interpreter','latex')
% end
% end
% exportgraphics(gcf,'figure/trim_condition.png','resolution',600)

%%
figure('units','centimeters','position',[0 2 24 15]);
for j = 1:num_alt
% for k = 1:num_mor
    subplot(1,3,j)
    colororder({'b','r'})
    alt = alt_grid(j); % trim alt [ft]
    tas = sas2tas(sas_grid,alt); % [kts]
    yyaxis right
    plot([0 0],[0 0],'-.xk',[0 0],[0 0],'--k',[0 0],[0 0],'-ok'), hold on
    plot(tas,squeeze(sch_param.sch_throttle_trim.data.D(:,:,:,j,1)),'-.xr'), hold on
    plot(tas,squeeze(sch_param.sch_throttle_trim.data.D(:,:,:,j,2)),'--r'), hold on
    plot(tas,squeeze(sch_param.sch_throttle_trim.data.D(:,:,:,j,3)),'-or'), hold on
    ylabel('Throttle [%]')
    ylim([29 58]) 

    yyaxis left
    plot([0 0],[0 0],'-.xk',[0 0],[0 0],'--k',[0 0],[0 0],'-ok'), hold on
    plot(tas,squeeze(sch_param.sch_alp_trim.data.D(:,:,:,j,1)),'-.xb'), hold on
    plot(tas,squeeze(sch_param.sch_alp_trim.data.D(:,:,:,j,2)),'--b'), hold on
    plot(tas,squeeze(sch_param.sch_alp_trim.data.D(:,:,:,j,3)),'-ob'), hold on
    ylabel('Angle of Attack [deg]')
    ylim([1 12.5]) 
    grid on, grid minor
    xlabel('True Airspeed [kt]')
    title(['h = ',num2str(alt),' ft'],'interpreter','latex')
    xlim([60 120])
    legend('\eta_s = -25%','\eta_s = -12.5%','\eta_s = 0%','location','best')
% end
end
exportgraphics(gcf,'figure/trim_condition.png','resolution',600)
