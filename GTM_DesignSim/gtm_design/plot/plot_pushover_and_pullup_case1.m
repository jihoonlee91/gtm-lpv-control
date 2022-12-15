%% Nonlinear simulation
FlightMode = 2.1;
ControlMode = 2;
etas0 = 0; % [%]
etas1 = -25;
tas0  = 60; % [kt]
deltc = 90;
t_final = 10;
alt0  = 1000; % [ft]

phi_to_eta = 0;
RateofClimb = 0;
etasc_feedthrough = 1;

set_param('gtm_design/Guidance/CAS Maneuver/Push-over and Pull-up','commented','off')
set_param('gtm_design/Guidance/CAS Maneuver/High-g Turn','commented','on')
MWS               = init_design('GTM',1); % sensor noise on
MWS.bias.morphing = etas0; % set morphing parameter
MWS.StatesInp(9)  = alt0; % set altitude [ft]
target            = struct('gamma',0,'tas',tas0); % start trim condition target [knot]
set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','on') % control off
set_param('gtm_design/Guidance','commented','on')
set_param('gtm_design/Monitor','commented','on')
set_param('gtm_design','MinStep','auto')
loadmws(MWS,'gtm_design'); % load MWS
[MWS,Xtrim,Trimcond,Err] = trimgtm_morph_try(tas0,alt0,etas0);
MWS.SurfaceDeadbandOn = 1; % deadband off
loadmws(MWS,'gtm_design'); % load MWS

set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','off') % control on
set_param('gtm_design/Guidance','commented','off')
set_param('gtm_design/Monitor','commented','off')
set_param('gtm_design','MaxStep','auto')
set_param('gtm_design','RelTol','1e-3')
set_param('gtm_design','MinStep','0.004')

set_param('gtm_design/Guidance/Autopilot Step','commented','on')
set_param('gtm_design/Guidance/Autopilot Trajectory Tracking','commented','on')
set_param('gtm_design/Guidance/Heading Control','commented','on')
set_param('gtm_design/Guidance/CAS Maneuver','commented','off')

[tsim1,~,ysim1]=sim('gtm_design',[0 t_final]);

%%
eas_kt_1  = ysim1(:,1); % [knot]
tas_kt_1  = ysim1(:,2); % [knot]
alp_deg_1 = ysim1(:,3); % [deg]
bet_deg_1 = ysim1(:,4); % [deg]
gam_deg_1 = ysim1(:,5); % [deg]
chi_deg_1 = ysim1(:,6); % [deg]
u_ft_1    = ysim1(:,7); % [ft/s]
v_ft_1    = ysim1(:,8); % [ft/s]
w_ft_1    = ysim1(:,9); % [ft/s]
p_rad_1   = ysim1(:,10); % [rad/s]
q_rad_1   = ysim1(:,11); % [rad/s]
r_rad_1   = ysim1(:,12); % [rad/s]
lat_rad_1 = ysim1(:,13); % [rad]
lon_rad_1 = ysim1(:,14); % [rad]
alt_ft_1  = ysim1(:,15); % [ft]
phi_rad_1 = ysim1(:,16); % [rad]
the_rad_1 = ysim1(:,17); % [rad]
psi_rad_1 = ysim1(:,18); % [rad]
Ax_g_1    = ysim1(:,19); % [g]
Ay_g_1    = ysim1(:,20); % [g]
Az_g_1    = ysim1(:,21); % [g]

tas_c_kt_1       = command(:,1); % 
alt_c_ft_1       = command(:,2); % 
bet_c_deg_1      = command(:,3); % 
phi_c_deg_1      = command(:,4); % 
an_c_g_1         = command(:,5); % 
p_c_deg_1        = command(:,6); % 
etas_c_percent_1 = command(:,7); % 

p_N_ft_1 = NED(:,1);
p_E_ft_1 = NED(:,2);
p_D_ft_1 = NED(:,3);

deltc_1 = control(:,1);
delec_1 = control(:,2);
delac_1 = control(:,3);
delrc_1 = control(:,4);
etalc_1 = control(:,5);
etarc_1 = control(:,6);
etaac_1 = (etarc_1-etalc_1)/2;
etasc_1 = (etarc_1+etalc_1)/2;

delt_1 = Throttle;
dele_1 = SurfacePos.ELLOB.Data;
dela_1 = SurfacePos.AILR.Data;
delr_1 = SurfacePos.RUDL.Data;
etal_1 = SurfacePos.FLAPLOB.Data;
etar_1 = SurfacePos.FLAPROB.Data;
etaa_1 = (etar_1-etal_1)/2;
etas_1 = (etar_1+etal_1)/2;

%% Nonlinear simulation
ControlMode = 1;
[tsim2,~,ysim2]=sim('gtm_design',[0 t_final]);

%%
eas_kt_2  = ysim2(:,1); % [knot]
tas_kt_2  = ysim2(:,2); % [knot]
alp_deg_2 = ysim2(:,3); % [deg]
bet_deg_2 = ysim2(:,4); % [deg]
gam_deg_2 = ysim2(:,5); % [deg]
chi_deg_2 = ysim2(:,6); % [deg]
u_ft_2    = ysim2(:,7); % [ft/s]
v_ft_2    = ysim2(:,8); % [ft/s]
w_ft_2    = ysim2(:,9); % [ft/s]
p_rad_2   = ysim2(:,10); % [rad/s]
q_rad_2   = ysim2(:,11); % [rad/s]
r_rad_2   = ysim2(:,12); % [rad/s]
lat_rad_2 = ysim2(:,13); % [rad]
lon_rad_2 = ysim2(:,14); % [rad]
alt_ft_2  = ysim2(:,15); % [ft]
phi_rad_2 = ysim2(:,16); % [rad]
the_rad_2 = ysim2(:,17); % [rad]
psi_rad_2 = ysim2(:,18); % [rad]
Ax_g_2    = ysim2(:,19); % [g]
Ay_g_2    = ysim2(:,20); % [g]
Az_g_2    = ysim2(:,21); % [g]

tas_c_kt_2       = command(:,1); % 
alt_c_ft_2       = command(:,2); % 
bet_c_deg_2      = command(:,3); % 
phi_c_deg_2      = command(:,4); % 
an_c_g_2         = command(:,5); % 
p_c_deg_2        = command(:,6); % 
etas_c_percent_2 = command(:,7); % 

p_N_ft_2 = NED(:,1);
p_E_ft_2 = NED(:,2);
p_D_ft_2 = NED(:,3);

deltc_2 = control(:,1);
delec_2 = control(:,2);
delac_2 = control(:,3);
delrc_2 = control(:,4);
etalc_2 = control(:,5);
etarc_2 = control(:,6);
etaac_2 = (etarc_2-etalc_2)/2;
etasc_2 = (etarc_2+etalc_2)/2;

delt_2 = Throttle;
dele_2 = SurfacePos.ELLOB.Data;
dela_2 = SurfacePos.AILR.Data;
delr_2 = SurfacePos.RUDL.Data;
etal_2 = SurfacePos.FLAPLOB.Data;
etar_2 = SurfacePos.FLAPROB.Data;
etaa_2 = (etar_2-etal_2)/2;
etas_2 = (etar_2+etal_2)/2;

%% Nonlinear simulation
ControlMode = 5;
[tsim3,~,ysim3]=sim('gtm_design',[0 t_final]);

%%
eas_kt_3  = ysim3(:,1); % [knot]
tas_kt_3  = ysim3(:,2); % [knot]
alp_deg_3 = ysim3(:,3); % [deg]
bet_deg_3 = ysim3(:,4); % [deg]
gam_deg_3 = ysim3(:,5); % [deg]
chi_deg_3 = ysim3(:,6); % [deg]
u_ft_3    = ysim3(:,7); % [ft/s]
v_ft_3    = ysim3(:,8); % [ft/s]
w_ft_3    = ysim3(:,9); % [ft/s]
p_rad_3   = ysim3(:,10); % [rad/s]
q_rad_3   = ysim3(:,11); % [rad/s]
r_rad_3   = ysim3(:,12); % [rad/s]
lat_rad_3 = ysim3(:,13); % [rad]
lon_rad_3 = ysim3(:,14); % [rad]
alt_ft_3  = ysim3(:,15); % [ft]
phi_rad_3 = ysim3(:,16); % [rad]
the_rad_3 = ysim3(:,17); % [rad]
psi_rad_3 = ysim3(:,18); % [rad]
Ax_g_3    = ysim3(:,19); % [g]
Ay_g_3    = ysim3(:,20); % [g]
Az_g_3    = ysim3(:,21); % [g]

tas_c_kt_3       = command(:,1); % 
alt_c_ft_3       = command(:,2); % 
bet_c_deg_3      = command(:,3); % 
phi_c_deg_3      = command(:,4); % 
an_c_g_3         = command(:,5); % 
p_c_deg_3        = command(:,6); % 
etas_c_percent_3 = command(:,7); % 

p_N_ft_3 = NED(:,1);
p_E_ft_3 = NED(:,2);
p_D_ft_3 = NED(:,3);

deltc_3 = control(:,1);
delec_3 = control(:,2);
delac_3 = control(:,3);
delrc_3 = control(:,4);
etalc_3 = control(:,5);
etarc_3 = control(:,6);
etaac_3 = (etarc_3-etalc_3)/2;
etasc_3 = (etarc_3+etalc_3)/2;

delt_3 = Throttle;
dele_3 = SurfacePos.ELLOB.Data;
dela_3 = SurfacePos.AILR.Data;
delr_3 = SurfacePos.RUDL.Data;
etal_3 = SurfacePos.FLAPLOB.Data;
etar_3 = SurfacePos.FLAPROB.Data;
etaa_3 = (etar_3-etal_3)/2;
etas_3 = (etar_3+etal_3)/2;

%%
figure('units','centimeters','position',[0 13 20 10.5]);
trajectory3(p_E_ft_1,p_N_ft_1,-p_D_ft_1,-phi_rad_1,the_rad_1,-psi_rad_1,1/100,floor(numel(tsim1)/5-1),'boeing_747',[90 0],'b','b');
xlabel('East [ft]'), ylabel('North [ft]'), zlabel('Elevation [ft]'), box on, grid on, grid minor, legend('LPV','H_\infty','H_2','location','northeast')
ylim([-200 1600]), zlim([250 1250])
exportgraphics(gcf,'figure/pushover_and_pullup_trajectory_case1_lpvu.png','resolution',300)

%%
figure('units','centimeters','position',[0 7 20 10.5]);
trajectory3(p_E_ft_2,p_N_ft_2,-p_D_ft_2,-phi_rad_2,the_rad_2,-psi_rad_2,1/100,floor(numel(tsim2)/5-1),'boeing_747',[90 0],'r','--r');
xlabel('East [ft]'), ylabel('North [ft]'), zlabel('Elevation [ft]'), box on, grid on, grid minor, legend('LPV','H_\infty','H_2','location','northeast')
ylim([-200 1600]), zlim([250 1250])
exportgraphics(gcf,'figure/pushover_and_pullup_trajectory_case1_hinf.png','resolution',300)

%%
figure('units','centimeters','position',[0 2 20 10.5]);
trajectory3(p_E_ft_3,p_N_ft_3,-p_D_ft_3,-phi_rad_3,the_rad_3,-psi_rad_3,1/100,floor(numel(tsim3)/5-1),'boeing_747',[90 0],'g',':g');
xlabel('East [ft]'), ylabel('North [ft]'), zlabel('Elevation [ft]'), box on, grid on, grid minor, legend('LPV','H_\infty','H_2','location','northeast')
ylim([-200 1600]), zlim([250 1250])
exportgraphics(gcf,'figure/pushover_and_pullup_trajectory_case1_h2op.png','resolution',300)

%%
figure('units','centimeters','position',[13 2 20 12]);
subplot(3,1,1), plot(tsim1,-Az_g_1,'b',tsim2,-Az_g_2,'--r',tsim3,-Az_g_3,':g',tsim2,an_c_g_2,'-.k'),              xlabel('Time [sec]'), grid on, grid minor, legend('LPV','H_\infty','H_2','Command','location','southeast','numcolumns',2), ylim padded, xlim tight, ylabel('Normal Acceleration [g]')
subplot(3,1,2), plot(tsim1,p_rad_1*r2d,'b',tsim2,p_rad_2*r2d,'--r',tsim3,p_rad_3*r2d,':g',tsim2,p_c_deg_2,'-.k'), xlabel('Time [sec]'), grid on, grid minor, ylim padded, xlim tight, ylabel('Roll Rate [deg/s]')
subplot(3,1,3), plot(tsim1,etas_1,'k',tsim1,etasc_1,'-.k'),                                                       xlabel('Time [sec]'), grid on, grid minor, legend('\eta_s','\eta_{s_c}','location','east'), ylim padded, xlim tight, ylabel('Sym. Morphing [%]')
exportgraphics(gcf,'figure/pushover_and_pullup_command_case1.png','resolution',300)

%%
figure('units','centimeters','position',[13*2 2 20 16]);
subplot(4,1,1), plot(tsim1,tas_kt_1,'b',tsim2,tas_kt_2,'--r',tsim3,tas_kt_3,':g'),           xlabel('Time [sec]'), grid on, grid minor, legend('LPV','H_\infty','H_2','Command','location','southeast','numcolumns',3), ylim padded, xlim tight, ylabel('True Airspeed [kt]')
subplot(4,1,2), plot(tsim1,alt_ft_1,'b',tsim2,alt_ft_2,'--r',tsim3,alt_ft_3,':g'),           xlabel('Time [sec]'), grid on, grid minor, legend('LPV','H_\infty','H_2','Command','location','northeast','numcolumns',3), ylim padded, xlim tight, ylabel('Altitude [ft]')
subplot(4,1,3), plot(tsim1,alp_deg_1,'b',tsim2,alp_deg_2,'--r',tsim3,alp_deg_3,':g'),         xlabel('Time [sec]'), grid on, grid minor, legend('LPV','H_\infty','H_2','Command','location','southeast','numcolumns',3), ylim padded, xlim tight, ylabel('Angle of Attack [deg]')
subplot(4,1,4), plot(tsim1,phi_rad_1*r2d,'b',tsim2,phi_rad_2*r2d,'--r',tsim3,phi_rad_3*r2d,':g'), xlabel('Time [sec]'), grid on, grid minor, legend('LPV','H_\infty','H_2','Command','location','southeast','numcolumns',3), ylim padded, xlim tight, ylabel('Roll Angle [deg]')
exportgraphics(gcf,'figure/pushover_and_pullup_state_case1.png','resolution',300)

%%
figure('units','centimeters','position',[13*3 2 20 16]);
subplot(4,1,1), plot(tsim1,dela_1,'b',tsim1,delac_1,'-.b',tsim2,dela_2,'r',tsim2,delac_2,'-.r',tsim3,dela_3,'g',tsim3,delac_3,'-.g'), xlabel('Time [sec]'), grid on, grid minor, ylim padded, xlim tight, ylabel('Aileron [deg]')
subplot(4,1,2), plot(tsim1,dele_1,'b',tsim1,delec_1,'-.b',tsim2,dele_2,'r',tsim2,delec_2,'-.r',tsim3,dele_3,'g',tsim3,delec_3,'-.g'), xlabel('Time [sec]'), grid on, grid minor, ylim padded, xlim tight, ylabel('Elevator [deg]'), legend('LPV','LPV Cmd','H_\infty','H_\infty Cmd','H_2','H_2 Cmd','location','northeast','numcolumns',3,'fontsize',7)
subplot(4,1,3), plot(tsim1,delr_1,'b',tsim1,delrc_1,'-.b',tsim2,delr_2,'r',tsim2,delrc_2,'-.r',tsim3,delr_3,'g',tsim3,delrc_3,'-.g'), xlabel('Time [sec]'), grid on, grid minor, ylim padded, xlim tight, ylabel('Rudder [deg]')
subplot(4,1,4), plot(tsim1,etaa_1,'b',tsim1,etaac_1,'-.b',tsim2,etaa_2,'r',tsim2,etaac_2,'-.r',tsim3,etaa_3,'g',tsim3,etaac_3,'-.g'), xlabel('Time [sec]'), grid on, grid minor, ylim padded, xlim tight, ylabel('Asy. Morphing [%]')
exportgraphics(gcf,'figure/pushover_and_pullup_input_case1.png','resolution',300)
