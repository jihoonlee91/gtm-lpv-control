figure('units','centimeters','position',[0 2 13 10])
for i = 1:num_sas
for j = 1:num_alt
    sas = sas_grid(i); % trim sas [kt]
    alt = alt_grid(j); % trim alt [ft]
    tas = sas2tas(sas,alt); % [kts]
    yyaxis left
    plot(tas*ones(num_mor),squeeze(alp_trim(i,j,:)),'-o','MarkerFaceColor',[0 0.4470 0.7410]), hold on
    ylabel('Angle of Attack [deg]')
    yyaxis right
    plot(tas*ones(num_mor),squeeze(throttle_trim(i,j,:)),'--o')
    ylabel('Throttle [%]')
    throttle_trim(i,j,1)
    throttle_trim(i,j,2)
    throttle_trim(i,j,3)
end
end
grid on, grid minor
xlabel('True Airspeed [kt]')
exportgraphics(gcf,'figure/trim_condition.png','resolution',300)

