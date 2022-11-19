fig = figure('units','centimeters','position',[0 2 13 10]);
[sas_mesh,alt_mesh] = meshgrid(sas_grid,alt_grid);
tas_mesh = zeros(size(sas_mesh));
for i = 1:num_sas
for j = 1:num_alt
    tas_mesh(j,i) = sas2tas(sas_grid(i),alt_grid(j));
end
end
scatter(tas_mesh(:),alt_mesh(:),'filled'), hold on
scatter(sas_mesh(:),alt_mesh(:))
xlabel('Airspeed [kt]'), ylabel('Altitude [ft]')
legend('True Airspeed','Synthetic Airspeed','location','best')
grid on, grid minor
xlim([50 100])
ylim([-1000 11000])
box on
exportgraphics(fig,'figure/parameter_grid.png','resolution',300)
% savefig(fig,'figure/parameter_grid.fig')

