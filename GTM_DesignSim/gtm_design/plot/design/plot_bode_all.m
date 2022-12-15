%% cas_lon_bode_an
plot_bode(CLlon2lpvu,CLlon2hinf,CLlon2h2op,Wlon2,1,3,{2*pi*1e-2,2*pi*1e2})
exportgraphics(gcf,'figure/cas_lon_bode_an.png','resolution',300)

%% cas_lat_bode_bet
plot_bode(CLlat3lpvu,CLlat3hinf,CLlat3h2op,Wlat3,1,1,{2*pi*1e-2,2*pi*1e2})
exportgraphics(gcf,'figure/cas_lat_bode_bet.png','resolution',300)

%% cas_lat_bode_p
plot_bode(CLlat3lpvu,CLlat3hinf,CLlat3h2op,Wlat3,2,2,{2*pi*1e-2,2*pi*1e2})
exportgraphics(gcf,'figure/cas_lat_bode_p.png','resolution',300)

%% auto_lon_bode_tas
plot_bode(CLlon5lpvu,CLlon5hinf,CLlon5h2op,Wlon5,1,1,{2*pi*1e-2,2*pi*1e2})
exportgraphics(gcf,'figure/auto_lon_bode_tas.png','resolution',300)

%% auto_lon_bode_alt
plot_bode(CLlon5lpvu,CLlon5hinf,CLlon5h2op,Wlon5,2,5,{2*pi*1e-2,2*pi*1e2})
exportgraphics(gcf,'figure/auto_lon_bode_alt.png','resolution',300)

%% auto_lat_bode_bet
plot_bode(CLlat4lpvu,CLlat4hinf,CLlat4h2op,Wlat4,1,1,{2*pi*1e-2,2*pi*1e2})
exportgraphics(gcf,'figure/auto_lat_bode_bet.png','resolution',300)

%% auto_lat_bode_p
plot_bode(CLlat4lpvu,CLlat4hinf,CLlat4h2op,Wlat4,2,4,{2*pi*1e-2,2*pi*1e2})
exportgraphics(gcf,'figure/auto_lat_bode_p.png','resolution',300)
