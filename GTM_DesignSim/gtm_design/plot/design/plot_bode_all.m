%% cas_lon_bode_an
plot_bode(CLlon2lpvu,CLlon2hinf,Wlon2,1,3,{2*pi*1e-4,2*pi*1e4})
exportgraphics(gcf,'figure/cas_lon_bode_an.png','resolution',300)

%% cas_lat_bode_bet
plot_bode(CLlat3lpvu,CLlat3hinf,Wlat3,1,1,{2*pi*1e-4,2*pi*1e4})
exportgraphics(gcf,'figure/cas_lat_bode_bet.png','resolution',300)

%% cas_lat_bode_p
plot_bode(CLlat3lpvu,CLlat3hinf,Wlat3,2,2,{2*pi*1e-4,2*pi*1e4})
exportgraphics(gcf,'figure/cas_lat_bode_p.png','resolution',300)

%% auto_lon_bode_tas
plot_bode(CLlon5lpvu,CLlon5hinf,Wlon5,1,1,{2*pi*1e-4,2*pi*1e4})
exportgraphics(gcf,'figure/auto_lon_bode_tas.png','resolution',300)

%% auto_lon_bode_alt
plot_bode(CLlon5lpvu,CLlon5hinf,Wlon5,2,5,{2*pi*1e-4,2*pi*1e4})
exportgraphics(gcf,'figure/auto_lon_bode_alt.png','resolution',300)

%% auto_lat_bode_bet
plot_bode(CLlat4lpvu,CLlat4hinf,Wlat4,1,1,{2*pi*1e-4,2*pi*1e4})
exportgraphics(gcf,'figure/auto_lat_bode_bet.png','resolution',300)

%% auto_lat_bode_p
plot_bode(CLlat4lpvu,CLlat4hinf,Wlat4,2,4,{2*pi*1e-4,2*pi*1e4})
exportgraphics(gcf,'figure/auto_lat_bode_p.png','resolution',300)
