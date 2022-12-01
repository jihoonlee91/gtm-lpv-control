set(0,'DefaultLineLineWidth',1)
figure('units','centimeters','position',[0 2 13 20]);
for i = 1:num_sas
for j = 1:num_alt
    sas = sas_grid(i); % trim sas [kt]
    alt = alt_grid(j); % trim alt [ft]
    tas = sas2tas(sas,alt); % [kts]
    yyaxis left
    plot(tas*ones(num_mor,1),squeeze(alp_trim(i,j,:)),'-'), hold on
    scatter(tas*ones(num_mor,1),squeeze(alp_trim(i,j,:)),linspace(30,90,num_mor),'filled','o')
    ylabel('Angle of Attack [deg]')
    ylim padded
    yyaxis right
    plot(tas*ones(num_mor,1),squeeze(throttle_trim(i,j,:)),'--')
    scatter(tas*ones(num_mor,1),squeeze(throttle_trim(i,j,:)),linspace(30,90,num_mor),'o')
    ylabel('Throttle [%]')
    ylim padded
end
end
grid on, grid minor
xlabel('True Airspeed [kt]')
exportgraphics(gcf,'figure/trim_condition.png','resolution',300)
