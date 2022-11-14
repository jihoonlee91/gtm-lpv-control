figure('units','centimeters','position',[0 2 13 10])
h = pzplot(LTI_lon); grid on, axis equal
xlim([-15 10])
axes('position',[.66 .175 .22 .25])
lon_real = real(eig(LTI_lon));
lon_imag = imag(eig(LTI_lon));
lon_real_vec = lon_real(:);
lon_imag_vec = lon_imag(:);
lon_real_vec_sort = lon_real_vec(lon_real_vec>-0.1);
lon_imag_vec_sort = lon_imag_vec(lon_real_vec>-0.1);
scatter(lon_real_vec_sort(1:num_sas*num_alt*num_mor*2),lon_imag_vec_sort(1:num_sas*num_alt*num_mor*2),'x')
xlim([-1.5*max(abs(lon_real_vec_sort)) 0])
ylim(1.5*max(abs(lon_imag_vec_sort))*[-1 1])
box on % put box around new pair of axes
grid on
grid minor
p = getoptions(h);
p.Title.String = '';
setoptions(h,p)
exportgraphics(gcf,'figure/mode_lon.png','resolution',300)

figure('units','centimeters','position',[13 2 13 10])
h = pzplot(LTI_lat); grid on, axis equal
xlim([-15 10])
axes('position',[.66 .175 .22 .25])
lat_real = real(eig(LTI_lat));
lat_imag = imag(eig(LTI_lat));
lat_real_vec = lat_real(:);
lat_imag_vec = lat_imag(:);
lat_real_vec_sort = lat_real_vec(abs(lat_imag_vec)<0.1);
lat_imag_vec_sort = lat_imag_vec(abs(lat_imag_vec)<0.1);
scatter(lat_real_vec_sort(2:2:num_sas*num_alt*num_mor*2),lat_imag_vec_sort(2:2:num_sas*num_alt*num_mor*2),'x')
xlim([2.5*min((lat_real_vec_sort(2:2:num_sas*num_alt*num_mor*2))) 1.5*max(abs(lat_real_vec_sort(2:2:num_sas*num_alt*num_mor*2)))])
ylim([-1 1]), yticks(0)
box on % put box around new pair of axes
grid on
grid minor
p = getoptions(h);
p.Title.String = '';
setoptions(h,p)
exportgraphics(gcf,'figure/mode_lat.png','resolution',300)

