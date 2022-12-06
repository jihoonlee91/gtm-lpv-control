figure('units','centimeters','position',[0 13 20 15]);

scatter(100,100,'rx'), hold on
scatter(100,100,'gx')
scatter(100,100,'bx')

clearvars LTI_lon
k = 1;
for j = 1:num_alt
for i = 1:num_sas
    LTI_lon(:,:,i,j,k) = ss(Plon5.data(:,:,i,j,k).A(1:4,1:4),Plon5.data(:,:,i,j,k).B(1:4,:),Plon5.data(:,:,i,j,k).C(1:4,1:4),Plon5.data(:,:,i,j,k).D(1:4,:));
end
end
h = pzplot(LTI_lon,'r'); grid on, axis equal, hold on
legend('\eta_s = -25% (retracted)','\eta_s = -12.5% (medium)','\eta_s = 0% (extended)')

xlim([-15 10])
ylim([-10 10])
axes('position',[.66 .175 .22 .25])
lon_real = real(eig(LTI_lon));
lon_imag = imag(eig(LTI_lon));
lon_real_vec = lon_real(:);
lon_imag_vec = lon_imag(:);
lon_real_vec_sort = lon_real_vec(lon_real_vec>-0.1);
lon_imag_vec_sort = lon_imag_vec(lon_real_vec>-0.1);
scatter(lon_real_vec_sort(1:num_sas*num_alt*2),lon_imag_vec_sort(1:num_sas*num_alt*2),'xr')
% xlim([-1.2*max(abs(lon_real_vec_sort)) max((lon_real_vec_sort))+0.2*max(abs(lon_real_vec_sort))])
% ylim(1.2*max(abs(lon_imag_vec_sort))*[-1 1])
xlim([-0.045 0.005])
ylim([-0.5 0.5])
box on
grid on
grid minor
p = getoptions(h);
p.Title.String = '';
setoptions(h,p)
exportgraphics(gcf,'figure/pzmap_lon_1.png','resolution',300)

%%
figure('units','centimeters','position',[15 13 20 15]);

scatter(100,100,'rx'), hold on
scatter(100,100,'gx')
scatter(100,100,'bx')

clearvars LTI_lon
% for k = 1:num_mor 
k = 2;
for j = 1:num_alt
for i = 1:num_sas
    LTI_lon(:,:,i,j,k) = ss(Plon5.data(:,:,i,j,k).A(1:4,1:4),Plon5.data(:,:,i,j,k).B(1:4,:),Plon5.data(:,:,i,j,k).C(1:4,1:4),Plon5.data(:,:,i,j,k).D(1:4,:));
end
end
% end

h = pzplot(LTI_lon,'g'); grid on, axis equal
legend('\eta_s = -25% (retracted)','\eta_s = -12.5% (medium)','\eta_s = 0% (extended)')

xlim([-15 10])
ylim([-10 10])
axes('position',[.66 .175 .22 .25])
lon_real = real(eig(LTI_lon));
lon_imag = imag(eig(LTI_lon));
lon_real_vec = lon_real(:);
lon_imag_vec = lon_imag(:);
lon_real_vec_sort = lon_real_vec(lon_real_vec>-0.1);
lon_imag_vec_sort = lon_imag_vec(lon_real_vec>-0.1);
scatter(lon_real_vec_sort(1:num_sas*num_alt*2),lon_imag_vec_sort(1:num_sas*num_alt*2),'xg')
% xlim([-1.2*max(abs(lon_real_vec_sort)) max((lon_real_vec_sort))+0.2*max(abs(lon_real_vec_sort))])
% ylim(1.2*max(abs(lon_imag_vec_sort))*[-1 1])
xlim([-0.045 0.005])
ylim([-0.5 0.5])
box on
grid on
grid minor
p = getoptions(h);
p.Title.String = '';
setoptions(h,p)
exportgraphics(gcf,'figure/pzmap_lon_2.png','resolution',300)

%%
figure('units','centimeters','position',[30 13 20 15]);

scatter(100,100,'rx'), hold on
scatter(100,100,'gx')
scatter(100,100,'bx')

clearvars LTI_lon
% for k = 1:num_mor 
k = 3;
for j = 1:num_alt
for i = 1:num_sas
    LTI_lon(:,:,i,j,k) = ss(Plon5.data(:,:,i,j,k).A(1:4,1:4),Plon5.data(:,:,i,j,k).B(1:4,:),Plon5.data(:,:,i,j,k).C(1:4,1:4),Plon5.data(:,:,i,j,k).D(1:4,:));
end
end
% end

h = pzplot(LTI_lon,'b'); grid on, axis equal
legend('\eta_s = -25% (retracted)','\eta_s = -12.5% (medium)','\eta_s = 0% (extended)')

xlim([-15 10])
ylim([-10 10])
axes('position',[.66 .175 .22 .25])
lon_real = real(eig(LTI_lon));
lon_imag = imag(eig(LTI_lon));
lon_real_vec = lon_real(:);
lon_imag_vec = lon_imag(:);
lon_real_vec_sort = lon_real_vec(lon_real_vec>-0.1);
lon_imag_vec_sort = lon_imag_vec(lon_real_vec>-0.1);
scatter(lon_real_vec_sort(1:num_sas*num_alt*2),lon_imag_vec_sort(1:num_sas*num_alt*2),'xb')
% xlim([-1.2*max(abs(lon_real_vec_sort)) max((lon_real_vec_sort))+0.2*max(abs(lon_real_vec_sort))])
% ylim(1.2*max(abs(lon_imag_vec_sort))*[-1 1])
xlim([-0.045 0.005])
ylim([-0.5 0.5])
box on
grid on
grid minor
p = getoptions(h);
p.Title.String = '';
setoptions(h,p)
exportgraphics(gcf,'figure/pzmap_lon_3.png','resolution',300)

%%
figure('units','centimeters','position',[0 2 20 15]);

scatter(100,100,'rx'), hold on
scatter(100,100,'gx')
scatter(100,100,'bx')

h = pzplot(Plat4.data(:,:,:,:,1),'r'); grid on, axis equal
legend('\eta_s = -25% (retracted)','\eta_s = -12.5% (medium)','\eta_s = 0% (extended)')

xlim([-15 10])
ylim([-10 10])
axes('position',[.66 .175 .22 .25])
lat_real = real(eig(Plat4.data(:,:,:,:,1)));
lat_imag = imag(eig(Plat4.data(:,:,:,:,1)));
lat_real_vec = lat_real(:);
lat_imag_vec = lat_imag(:);
lat_real_vec_sort = lat_real_vec(abs(lat_imag_vec)<0.1);
lat_imag_vec_sort = lat_imag_vec(abs(lat_imag_vec)<0.1);
scatter(lat_real_vec_sort(2:2:num_sas*num_alt*2),lat_imag_vec_sort(2:2:num_sas*num_alt*2),'xr')
xlim([-0.15 0.3])
ylim([-1 1]), yticks(0)
box on
grid on
grid minor
p = getoptions(h);
p.Title.String = '';
setoptions(h,p)
exportgraphics(gcf,'figure/pzmap_lat_1.png','resolution',300)

%%
figure('units','centimeters','position',[15 2 20 15]);

scatter(100,100,'rx'), hold on
scatter(100,100,'gx')
scatter(100,100,'bx')

h = pzplot(Plat4.data(:,:,:,:,2),'g'); grid on, axis equal
legend('\eta_s = -25% (retracted)','\eta_s = -12.5% (medium)','\eta_s = 0% (extended)')

xlim([-15 10])
ylim([-10 10])
axes('position',[.66 .175 .22 .25])
lat_real = real(eig(Plat4.data(:,:,:,:,2)));
lat_imag = imag(eig(Plat4.data(:,:,:,:,2)));
lat_real_vec = lat_real(:);
lat_imag_vec = lat_imag(:);
lat_real_vec_sort = lat_real_vec(abs(lat_imag_vec)<0.1);
lat_imag_vec_sort = lat_imag_vec(abs(lat_imag_vec)<0.1);
scatter(lat_real_vec_sort(2:2:num_sas*num_alt*2),lat_imag_vec_sort(2:2:num_sas*num_alt*2),'xg')
xlim([-0.15 0.3])
ylim([-1 1]), yticks(0)
box on
grid on
grid minor
p = getoptions(h);
p.Title.String = '';
setoptions(h,p)
exportgraphics(gcf,'figure/pzmap_lat_2.png','resolution',300)

%%
figure('units','centimeters','position',[30 2 20 15]);

scatter(100,100,'rx'), hold on
scatter(100,100,'gx')
scatter(100,100,'bx')

h = pzplot(Plat4.data(:,:,:,:,3),'b'); grid on, axis equal
legend('\eta_s = -25% (retracted)','\eta_s = -12.5% (medium)','\eta_s = 0% (extended)')

xlim([-15 10])
ylim([-10 10])
axes('position',[.66 .175 .22 .25])
lat_real = real(eig(Plat4.data(:,:,:,:,3)));
lat_imag = imag(eig(Plat4.data(:,:,:,:,3)));
lat_real_vec = lat_real(:);
lat_imag_vec = lat_imag(:);
lat_real_vec_sort = lat_real_vec(abs(lat_imag_vec)<0.1);
lat_imag_vec_sort = lat_imag_vec(abs(lat_imag_vec)<0.1);
scatter(lat_real_vec_sort(2:2:num_sas*num_alt*2),lat_imag_vec_sort(2:2:num_sas*num_alt*2),'xb')
xlim([-0.15 0.3])
ylim([-1 1]), yticks(0)
box on
grid on
grid minor
p = getoptions(h);
p.Title.String = '';
setoptions(h,p)
exportgraphics(gcf,'figure/pzmap_lat_3.png','resolution',300)
