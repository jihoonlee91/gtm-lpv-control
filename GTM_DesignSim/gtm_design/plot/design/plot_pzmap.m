figure('units','centimeters','position',[0 2 20 15]);

clearvars LTI_lon
for k = 1:num_mor 
for j = 1:num_alt
for i = 1:num_sas
    LTI_lon(:,:,i,j,k) = ss(Plon5.data(:,:,i,j,k).A(1:4,1:4),Plon5.data(:,:,i,j,k).B(1:4,:),Plon5.data(:,:,i,j,k).C(1:4,1:4),Plon5.data(:,:,i,j,k).D(1:4,:));
end
end
end

h = pzplot(LTI_lon,'b'); grid on, axis equal
xlim([-15 10])
axes('position',[.66 .175 .22 .25])
lon_real = real(eig(LTI_lon));
lon_imag = imag(eig(LTI_lon));
lon_real_vec = lon_real(:);
lon_imag_vec = lon_imag(:);
lon_real_vec_sort = lon_real_vec(lon_real_vec>-0.1);
lon_imag_vec_sort = lon_imag_vec(lon_real_vec>-0.1);
scatter(lon_real_vec_sort(1:num_sas*num_alt*num_mor*2),lon_imag_vec_sort(1:num_sas*num_alt*num_mor*2),'xb')
xlim([-1.2*max(abs(lon_real_vec_sort)) max((lon_real_vec_sort))+0.2*max(abs(lon_real_vec_sort))])
ylim(1.2*max(abs(lon_imag_vec_sort))*[-1 1])
box on
grid on
grid minor
p = getoptions(h);
p.Title.String = '';
setoptions(h,p)
exportgraphics(gcf,'figure/pzmap_lon.png','resolution',300)

figure('units','centimeters','position',[13 2 20 15]);
h = pzplot(Plat4.data,'b'); grid on, axis equal
xlim([-15 10])
axes('position',[.66 .175 .22 .25])
lat_real = real(eig(Plat4.data));
lat_imag = imag(eig(Plat4.data));
lat_real_vec = lat_real(:);
lat_imag_vec = lat_imag(:);
lat_real_vec_sort = lat_real_vec(abs(lat_imag_vec)<0.1);
lat_imag_vec_sort = lat_imag_vec(abs(lat_imag_vec)<0.1);
scatter(lat_real_vec_sort(2:2:num_sas*num_alt*num_mor*2),lat_imag_vec_sort(2:2:num_sas*num_alt*num_mor*2),'xb')
xlim([min((lat_real_vec_sort(2:2:num_sas*num_alt*num_mor*2)))-0.2*max(abs(lat_real_vec_sort(2:2:num_sas*num_alt*num_mor*2))) 1.2*max(abs(lat_real_vec_sort(2:2:num_sas*num_alt*num_mor*2)))])
ylim([-1 1]), yticks(0)
box on
grid on
grid minor
p = getoptions(h);
p.Title.String = '';
setoptions(h,p)
exportgraphics(gcf,'figure/pzmap_lat.png','resolution',300)
