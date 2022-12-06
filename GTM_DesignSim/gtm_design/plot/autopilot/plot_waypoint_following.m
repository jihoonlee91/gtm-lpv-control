%% Nonlinear simulation
FlightMode = 4.1;
ControlMode = 4;
phi_to_eta = 12.5/30;
tas0  = 75; % [kt]
t_final = 90;
LookaheadDistance = 1200;
TransitionRadius = 1000*0.3048;
alt0  = 1000; % [ft]
Waypoints = [0 0 -alt0;
             3000 0 -alt0;
             3000 3000 -alt0;
             0 3000 -alt0;
             0 0 -alt0;
             3000 0 -alt0];
phi_lim = 30;
etasc_feedthrough = 1;
etas0 = -12.5; % [%]

RateofClimb = 0;
etas1 = 0;
deltc = MWS.bias.throttle;
Radius = 2000; % not used
TurnDirection = 1; % not used

set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','on') % control off
set_param('gtm_design/Guidance/CAS Maneuver','commented','on')
set_param('gtm_design/Guidance','commented','on')
set_param('gtm_design/Monitor','commented','on')
set_param('gtm_design','MinStep','auto')

[MWS,Xtrim,Trimcond,Err] = trimgtm_morph_try(tas0,alt0,etas0);
MWS = NoiseParams_T2(MWS,1); % noise
MWS.SurfaceDeadbandOn = 1; % deadband
loadmws(MWS,'gtm_design');

set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','off') % control on
set_param('gtm_design/Guidance','commented','off')
set_param('gtm_design/Monitor','commented','off')
set_param('gtm_design','MinStep',num2str(MWS.Timestep-0.00001))
set_param('gtm_design/Guidance/Autopilot Step','commented','on')
set_param('gtm_design/Guidance/Autopilot Trajectory Tracking','commented','off')
set_param('gtm_design/Guidance/Heading Control','commented','off')

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

%%
ControlMode = 3;
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

%%
figure('units','centimeters','position',[10 2 20 19]);
trajectory3(p_E_ft_1,p_N_ft_1,-p_D_ft_1,-phi_rad_1,the_rad_1,-psi_rad_1,1/150,1500,'boeing_747',[90 90],'blue','blue');
xlim([-600 3600]), ylim([-600 3600]), plot3([0 0 3000 3000 0],[0 3000 3000 0 0],[1000 1000 1000 1000 1000],'-.ok')
xlabel('East [ft]'), ylabel('North [ft]'), zlabel('Elevation [ft]'), box on, grid on, grid minor, legend('LPV','H_\infty','Waypoint','location','northeast')
exportgraphics(gcf,'figure/waypoint_following_trajectory_top_lpvu.png','resolution',300)

%%
figure('units','centimeters','position',[15 2 20 19]);
trajectory3(p_E_ft_2,p_N_ft_2,-p_D_ft_2,-phi_rad_2,the_rad_2,-psi_rad_2,1/150,1500,'boeing_747',[90 90],'red','red');
xlim([-600 3600]), ylim([-600 3600]), plot3([0 0 3000 3000 0],[0 3000 3000 0 0],[1000 1000 1000 1000 1000],'-.ok')
xlabel('East [ft]'), ylabel('North [ft]'), zlabel('Elevation [ft]'), box on, grid on, grid minor, legend('LPV','H_\infty','Waypoint','location','northeast')
exportgraphics(gcf,'figure/waypoint_following_trajectory_top_hinf.png','resolution',300)

%%
figure('units','centimeters','position',[20 2 20 16]);
subplot(4,1,1), plot(tsim1,tas_kt_1,'b',tsim2,tas_kt_2,'--r',tsim1,tas_c_kt_1,'-.k'), xlabel('Time [sec]'), grid on, grid minor, ylim padded, xlim([0 t_final]), legend('LPV','H_\infty','Command','location','best'), ylabel('True Airspeed [kt]')
subplot(4,1,2), plot(tsim1,alt_ft_1,'b',tsim2,alt_ft_2,'--r',tsim1,alt_c_ft_1,'-.k'), xlabel('Time [sec]'), grid on, grid minor, ylim padded, xlim([0 t_final]), legend('LPV','H_\infty','Command','location','best'), ylabel('Altitude [ft]')
subplot(4,1,3), plot(tsim1,bet_deg_1,'b',tsim2,bet_deg_2,'--r',tsim1,tsim1*0,'-.k'), xlabel('Time [sec]'), grid on, grid minor, ylim padded, xlim([0 t_final]), legend('LPV','H_\infty','Command','location','best'), ylabel('Sideslip [deg]')
subplot(4,1,4), plot(tsim1,phi_rad_1*180/pi,'b',tsim1,phi_c_deg_1,'--b',tsim2,phi_rad_2*180/pi,'r',tsim2,phi_c_deg_2,'--r'), xlabel('Time [sec]'), grid on, grid minor, ylim padded, xlim([0 t_final]), legend('LPV (Actual)','LPV (Command)','H_\infty (Actual)','H_\infty (Command)','location','best'), ylabel('Roll [deg]')
exportgraphics(gcf,'figure/waypoint_following_command.png','resolution',300)

%%
figure('units','centimeters','position',[15 2 20 24]);
subplot(6,1,1), plot(tsim1,dela_1,'b',tsim1,delac_1,'--b',tsim2,dela_2,'r',tsim2,delac_2,'--r'), xlabel('Time [sec]'), grid on, grid minor, ylim padded, xlim([0 t_final]), legend('Actual (LPV)','Command (LPV)','Actual (H_\infty)','Command (H_\infty)','location','best'), ylabel('Aileron [deg]')
subplot(6,1,2), plot(tsim1,dele_1,'b',tsim1,delec_1,'--b',tsim2,dele_2,'r',tsim2,delec_2,'--r'), xlabel('Time [sec]'), grid on, grid minor, ylim padded, xlim([0 t_final]), legend('Actual (LPV)','Command (LPV)','Actual (H_\infty)','Command (H_\infty)','location','best'), ylabel('Elevator [deg]')
subplot(6,1,3), plot(tsim1,delr_1,'b',tsim1,delrc_1,'--b',tsim2,delr_2,'r',tsim2,delrc_2,'--r'), xlabel('Time [sec]'), grid on, grid minor, ylim padded, xlim([0 t_final]), legend('Actual (LPV)','Command (LPV)','Actual (H_\infty)','Command (H_\infty)','location','best'), ylabel('Rudder [deg]')
subplot(6,1,4), plot(tsim1,delt_1,'b',tsim1,deltc_1,'--b',tsim2,delt_2,'r',tsim2,deltc_2,'--r'), xlabel('Time [sec]'), grid on, grid minor, ylim padded, xlim([0 t_final]), legend('Actual (LPV)','Command (LPV)','Actual (H_\infty)','Command (H_\infty)','location','best'), ylabel('Throttle [%]')
subplot(6,1,5), plot(tsim1,etas_1,'k',tsim1,etasc_1,'--k'), xlabel('Time [sec]'), grid on, grid minor, ylim padded, xlim([0 t_final]), legend('Actual','Command','location','best'), ylabel('Sym. Morphing [%]')
subplot(6,1,6), plot(tsim1,etaa_1,'b',tsim1,etaac_1,'--b',tsim2,etaa_2,'r',tsim2,etaac_2,'--r'), xlabel('Time [sec]'), grid on, grid minor, ylim padded, xlim([0 t_final]), legend('Actual (LPV)','Command (LPV)','Actual (H_\infty)','Command (H_\infty)','location','best'), ylabel('Asy. Morphing [%]')
exportgraphics(gcf,'figure/waypoint_following_input.png','resolution',300)