set(0,'DefaultLineLineWidth',1)
figure('units','centimeters','position',[0 2 13 10]);
[sas_mesh,alt_mesh] = meshgrid(sas_grid,alt_grid);
tas_mesh = zeros(size(sas_mesh));
for i = 1:num_sas
for j = 1:num_alt
    tas_mesh(j,i) = sas2tas(sas_grid(i),alt_grid(j));
end
end
scatter(tas_mesh(:),alt_mesh(:),20,'b','filled'), hold on
scatter(sas_mesh(:),alt_mesh(:),'r')
xlabel('Airspeed [kt]'), ylabel('Altitude [ft]')
grid on, grid minor
xlim([min(sas_grid)-10 max(sas_grid)+10])
box on
ylim padded

tas_grid = sas2tas(sas_grid,alt_grid(end));
plot([sas_grid(1) sas_grid(1) sas_grid(end) sas_grid(end) sas_grid(1)],[alt_grid(1) alt_grid(end) alt_grid(end) alt_grid(1) alt_grid(1)],'color',[0.8500 0.3250 0.0980])
plot([sas_grid(1) tas_grid(1) tas_grid(end) sas_grid(end) sas_grid(1)],[alt_grid(1) alt_grid(end) alt_grid(end) alt_grid(1) alt_grid(1)],'--','color',[0 0.4470 0.7410])
legend('True Airspeed','Synthetic Airspeed','location','best')

exportgraphics(gcf,'figure/parameter_grid.png','resolution',600)
