%% Nonlinear simulation
FlightMode = 2.1;
ControlMode = 2;
etas0 = -25; % [%]
etas1 = -0;
tas0  = 60; % [kt]
deltc = 90;
t_final = 10;
alt0  = 1000; % [ft]

phi_to_eta = 0;

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
[MWS,Xtrim,Trimcond,Err] = trimgtm(target,'elev',1); % trim calculation
MWS.SurfaceDeadbandOn = 0; % deadband off
loadmws(MWS,'gtm_design'); % load MWS

set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','off') % control on
set_param('gtm_design/Guidance','commented','off')
set_param('gtm_design/Monitor','commented','off')
set_param('gtm_design','MaxStep','auto')
set_param('gtm_design','RelTol','1e-3')
set_param('gtm_design','MinStep',num2str(MWS.Timestep-0.00001))

set_param('gtm_design/Guidance/Autopilot Step','commented','on')
set_param('gtm_design/Guidance/Autopilot Trajectory Tracking','commented','on')
set_param('gtm_design/Guidance/Heading Control','commented','on')
set_param('gtm_design/Guidance/CAS Maneuver','commented','off')

[tsim,~,ysim]=sim('gtm_design',[0 t_final]);

%%
t_1       = tsim; % [sec]

eas_kt_1  = ysim(:,1); % [knot]
tas_kt_1  = ysim(:,2); % [knot]
alp_deg_1 = ysim(:,3); % [deg]
bet_deg_1 = ysim(:,4); % [deg]
gam_deg_1 = ysim(:,5); % [deg]
chi_deg_1 = ysim(:,6); % [deg]
u_ft_1    = ysim(:,7); % [ft/s]
v_ft_1    = ysim(:,8); % [ft/s]
w_ft_1    = ysim(:,9); % [ft/s]
p_rad_1   = ysim(:,10); % [rad/s]
q_rad_1   = ysim(:,11); % [rad/s]
r_rad_1   = ysim(:,12); % [rad/s]
lat_rad_1 = ysim(:,13); % [rad]
lon_rad_1 = ysim(:,14); % [rad]
alt_ft_1  = ysim(:,15); % [ft]
phi_rad_1 = ysim(:,16); % [rad]
the_rad_1 = ysim(:,17); % [rad]
psi_rad_1 = ysim(:,18); % [rad]
Ax_g_1    = ysim(:,19); % [g]
Ay_g_1    = ysim(:,20); % [g]
Az_g_1    = ysim(:,21); % [g]

tas_c_kt_1       = command(:,1); % 
alt_c_ft_1       = command(:,2); % 
bet_c_deg_1      = command(:,3); % 
phi_c_deg_1      = command(:,4); % 
an_c_g_1         = command(:,5); % 
p_c_deg_1        = command(:,6); % 
etas_c_percent_1 = command(:,7); % 

SurfacePos_1 = SurfacePos;
delt_percent_1 = Throttle;

p_N_ft_1 = NED(:,1);
p_E_ft_1 = NED(:,2);
p_D_ft_1 = NED(:,3);

%% Nonlinear simulation
etas0 = -0; % [%]
etas1 = -25;

[tsim,~,ysim]=sim('gtm_design',[0 t_final]);

%%
t_2       = tsim; % [sec]

eas_kt_2  = ysim(:,1); % [knot]
tas_kt_2  = ysim(:,2); % [knot]
alp_deg_2 = ysim(:,3); % [deg]
bet_deg_2 = ysim(:,4); % [deg]
gam_deg_2 = ysim(:,5); % [deg]
chi_deg_2 = ysim(:,6); % [deg]
u_ft_2    = ysim(:,7); % [ft/s]
v_ft_2    = ysim(:,8); % [ft/s]
w_ft_2    = ysim(:,9); % [ft/s]
p_rad_2   = ysim(:,10); % [rad/s]
q_rad_2   = ysim(:,11); % [rad/s]
r_rad_2   = ysim(:,12); % [rad/s]
lat_rad_2 = ysim(:,13); % [rad]
lon_rad_2 = ysim(:,14); % [rad]
alt_ft_2  = ysim(:,15); % [ft]
phi_rad_2 = ysim(:,16); % [rad]
the_rad_2 = ysim(:,17); % [rad]
psi_rad_2 = ysim(:,18); % [rad]
Ax_g_2    = ysim(:,19); % [g]
Ay_g_2    = ysim(:,20); % [g]
Az_g_2    = ysim(:,21); % [g]

tas_c_kt_2       = command(:,1); % 
alt_c_ft_2       = command(:,2); % 
bet_c_deg_2      = command(:,3); % 
phi_c_deg_2      = command(:,4); % 
an_c_g_2         = command(:,5); % 
p_c_deg_2        = command(:,6); % 
etas_c_percent_2 = command(:,7); % 

SurfacePos_2 = SurfacePos;
delt_percent_2  = Throttle;

p_N_ft_2 = NED(:,1);
p_E_ft_2 = NED(:,2);
p_D_ft_2 = NED(:,3);

%%
set(0,'DefaultLineLineWidth',1)

figure('units','centimeters','position',[0 2 13 10]);
trajectory3(p_E_ft_1,p_N_ft_1,-p_D_ft_1,-phi_rad_1,the_rad_1,-psi_rad_1,1/100,floor(min(numel(t_1),numel(t_2))/5-1),'boeing_747',[90 0],'blue','blue');
xlabel('East [ft]')
ylabel('North [ft]')
zlabel('Up [ft]')
box on
grid on
grid minor
legend('Configuration #1','Configuration #2','location','best')
view([45 30])
axis equal
view([90 0])
ylim([-200 1600])
zlim([0 1500])
exportgraphics(gcf,'figure/pushover_and_pullup_trajectory_1.png','resolution',300)

figure('units','centimeters','position',[13 2 13 10]);
trajectory3(p_E_ft_2,p_N_ft_2,-p_D_ft_2,-phi_rad_2,the_rad_2,-psi_rad_2,1/100,floor(min(numel(t_1),numel(t_2))/5-1),'boeing_747',[90 0],'red','red');
xlabel('East [ft]')
ylabel('North [ft]')
zlabel('Up [ft]')
box on
grid on
grid minor, hold on
legend('Configuration #1','Configuration #2','location','best')
view([45 30])
axis equal
view([90 0])
ylim([-200 1600])
zlim([0 1500])
exportgraphics(gcf,'figure/pushover_and_pullup_trajectory_2.png','resolution',300)

%%
figure('units','centimeters','position',[26 2 13 20]);
subplot(3,1,1), plot(t_1,-Az_g_1,'b',t_2,-Az_g_2,'--r',t_2,an_c_g_2,'-.'), ylabel('Normal Acceleration [g]'), xlabel('Time [sec]')
grid on, grid minor, hold on, legend('Configuration #1','Configuration #2','Command','location','best')
% ylim([-1 5])
subplot(3,1,2), plot(SurfacePos_1.FLAPLOB.Time,(SurfacePos_1.FLAPLOB.Data+SurfacePos_1.FLAPROB.Data)/2,'b',SurfacePos_2.FLAPLOB.Time,(SurfacePos_2.FLAPLOB.Data+SurfacePos_2.FLAPROB.Data)/2,'--r'), ylabel('Sym. Morphing [%]'), xlabel('Time [sec]')
grid on, grid minor, hold on, legend('Configuration #1','Configuration #2','location','best')
ylim([-30 5])
subplot(3,1,3), plot(SurfacePos_1.AILL.Time,SurfacePos_1.ELLOB.Data,'b',SurfacePos_2.AILL.Time,SurfacePos_2.ELLOB.Data,'--r'), ylabel('Elevator [deg]'), xlabel('Time [sec]')
grid on, grid minor, hold on, legend('Configuration #1','Configuration #2','location','best')
exportgraphics(gcf,'figure/pushover_and_pullup_command.png','resolution',300)

%%
figure('units','centimeters','position',[13*3 2 13 20]);
subplot(3,1,1), plot(t_1,tas_kt_1,'b',t_2,tas_kt_2,'--r'), ylabel('True Airspeed [kt]'), xlabel('Time [sec]')
grid on, grid minor, hold on, legend('Configuration #1','Configuration #2','location','best')
subplot(3,1,2), plot(t_1,alt_ft_1,'b',t_2,alt_ft_2,'--r'), ylabel('Altitude [ft]'), xlabel('Time [sec]')
grid on, grid minor, hold on, legend('Configuration #1','Configuration #2','location','best')
subplot(3,1,3), plot(t_1,alp_deg_1,'b',t_2,alp_deg_2,'--r'), ylabel('Angle of Attack [deg]'), xlabel('Time [sec]')
grid on, grid minor, hold on, legend('Configuration #1','Configuration #2','location','best')
exportgraphics(gcf,'figure/pushover_and_pullup_state.png','resolution',300)
