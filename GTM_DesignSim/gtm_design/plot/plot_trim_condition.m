fig = figure('units','centimeters','position',[0 2 13 20]);
for i = 1:num_sas
for j = 1:num_alt
    sas = sas_grid(i); % trim sas [kt]
    alt = alt_grid(j); % trim alt [ft]
    tas = sas2tas(sas,alt); % [kts]
    yyaxis left
%     plot(tas*ones(num_mor),squeeze(alp_trim(i,j,:)),'-o','MarkerFaceColor',[0 0.4470 0.7410]), hold on
    plot(tas*ones(num_mor,1),squeeze(alp_trim(i,j,:)),'-'), hold on
    scatter(tas*ones(num_mor,1),squeeze(alp_trim(i,j,:)),linspace(30,90,num_mor),'filled','o')
    ylabel('Angle of Attack [deg]')
    yyaxis right
    plot(tas*ones(num_mor,1),squeeze(throttle_trim(i,j,:)),'--')
    scatter(tas*ones(num_mor,1),squeeze(throttle_trim(i,j,:)),linspace(30,90,num_mor),'o')
    ylabel('Throttle [%]')
end
end
grid on, grid minor
xlabel('True Airspeed [kt]')
exportgraphics(fig,'figure/trim_condition.png','resolution',300)
% savefig(fig,'figure/trim_condition.fig')

clearvars fig