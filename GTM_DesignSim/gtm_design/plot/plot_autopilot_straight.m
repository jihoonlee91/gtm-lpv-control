%% Nonlinear simulation
FlightMode = 3.0;
ControlMode = 4;
% phi_to_eta = 12.5/45;
phi_to_eta = 0;

etas0 = -12.5; % [%]
tas0  = 75; % [kt]
deltc = MWS.bias.throttle;
t_final = 180;
LookaheadDistance = 700;
TransitionRadius = 1000*0.3048; % not used
Radius = 2000;
TurnDirection = 1;

alt0  = 1000; % [ft]
MWS               = init_design('GTM',1);
MWS.bias.morphing = etas0; % set morphing parameter
MWS.StatesInp(9)  = alt0; % set altitude [ft]
target            = struct('gamma',0,'tas',tas0); % start trim condition target [knot]
set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','on') % control off
set_param('gtm_design/Guidance/CAS Maneuver','commented','on')
set_param('gtm_design/Guidance','commented','on')
set_param('gtm_design/Monitor','commented','on')
set_param('gtm_design','MinStep','auto')
MWS.SurfaceDeadbandOn = 0;
loadmws(MWS,'gtm_design'); % load MWS
[MWS,Xtrim,Trimcond,Err] = trimgtm(target,'elev',1); % trim calculation
MWS.SurfaceDeadbandOn = 0;
loadmws(MWS,'gtm_design'); % load MWS

set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','off') % control on
set_param('gtm_design/Guidance','commented','off')
set_param('gtm_design/Monitor','commented','off')
set_param('gtm_design','MaxStep','auto')
set_param('gtm_design','RelTol','1e-3')
% set_param('gtm_design','MinStep','0.0049')
set_param('gtm_design','MinStep','0.0049')

% FlightMode
% Step: 1.0 (zero), 1.1 (normal acceleration), 1.2 (sideslip), 1.3 (roll rate)
% Maneuver: 2.1 (push-over and pull-up), 2.2 (high-g turn)
% Step: 3.0 (zero), 3.1 (airspeed), 3.2 (altitude), 3.3 (sideslip), 3.4 (roll)
% Trajectory: 4.1 (waypoint following), 4.2 (orbit following)

% ControlMode
% Off: 0
% Hinf CAS: 1
% LPV CAS: 2
% Hinf Autopilot: 3
% LPV Autopilot: 4

if FlightMode < 4
    set_param('gtm_design/Guidance/Autopilot Step','commented','off')
    set_param('gtm_design/Guidance/Autopilot Trajectory Tracking','commented','on')
    set_param('gtm_design/Guidance/Heading Control','commented','on')
else
    set_param('gtm_design/Guidance/Autopilot Step','commented','on')
    set_param('gtm_design/Guidance/Autopilot Trajectory Tracking','commented','off')
    set_param('gtm_design/Guidance/Heading Control','commented','off')
end
Waypoints = [0 0 -alt0;
             3000 0 -alt0;
             3000 3000 -alt0;
             0 3000 -alt0;
             0 0 -alt0];

set_param('gtm_design/Guidance/CAS Step','commented','on')

[tsim,xsim,ysim]=sim('gtm_design',[0 t_final]);
%%

set(0,'DefaultLineLineWidth',1)

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
% dele_deg_1      = SurfacePos(:,1);
% dela_deg_1      = SurfacePos(:,6);
% delr_deg_1      = SurfacePos(:,7);
% delml_percent_1 = SurfacePos(:,14);
% delmr_percent_1 = SurfacePos(:,16);
% delms_percent_1 = (delml_percent+delmr_percent)/2;
% delma_percent_1 = (delml_percent-delmr_percent)/2;
delt_percent_1  = Throttle;

p_N_ft_1 = NED(:,1);
p_E_ft_1 = NED(:,2);
p_D_ft_1 = NED(:,3);

%% Nonlinear simulation
FlightMode = 3.0;
ControlMode = 3;
% phi_to_eta = 12.5/45;
phi_to_eta = 0;

etas0 = -12.5; % [%]
tas0  = 75; % [kt]
deltc = MWS.bias.throttle;
LookaheadDistance = 700;
TransitionRadius = 1000*0.3048; % not used
Radius = 2000;
TurnDirection = 1;

alt0  = 1000; % [ft]
MWS               = init_design('GTM',1);
MWS.bias.morphing = etas0; % set morphing parameter
MWS.StatesInp(9)  = alt0; % set altitude [ft]
target            = struct('gamma',0,'tas',tas0); % start trim condition target [knot]
set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','on') % control off
set_param('gtm_design/Guidance/CAS Maneuver','commented','on')
set_param('gtm_design/Guidance','commented','on')
set_param('gtm_design/Monitor','commented','on')
set_param('gtm_design','MinStep','auto')
MWS.SurfaceDeadbandOn = 0;
loadmws(MWS,'gtm_design'); % load MWS
[MWS,Xtrim,Trimcond,Err] = trimgtm(target,'elev',1); % trim calculation
MWS.SurfaceDeadbandOn = 0;
loadmws(MWS,'gtm_design'); % load MWS

set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','off') % control on
set_param('gtm_design/Guidance','commented','off')
set_param('gtm_design/Monitor','commented','off')
set_param('gtm_design','MaxStep','auto')
set_param('gtm_design','RelTol','1e-3')
% set_param('gtm_design','MinStep','0.0049')
set_param('gtm_design','MinStep','0.0049')

% FlightMode
% Step: 1.0 (zero), 1.1 (normal acceleration), 1.2 (sideslip), 1.3 (roll rate)
% Maneuver: 2.1 (push-over and pull-up), 2.2 (high-g turn)
% Step: 3.0 (zero), 3.1 (airspeed), 3.2 (altitude), 3.3 (sideslip), 3.4 (roll)
% Trajectory: 4.1 (waypoint following), 4.2 (orbit following)

% ControlMode
% Off: 0
% Hinf CAS: 1
% LPV CAS: 2
% Hinf Autopilot: 3
% LPV Autopilot: 4

if FlightMode < 4
    set_param('gtm_design/Guidance/Autopilot Step','commented','off')
    set_param('gtm_design/Guidance/Autopilot Trajectory Tracking','commented','on')
    set_param('gtm_design/Guidance/Heading Control','commented','on')
else
    set_param('gtm_design/Guidance/Autopilot Step','commented','on')
    set_param('gtm_design/Guidance/Autopilot Trajectory Tracking','commented','off')
    set_param('gtm_design/Guidance/Heading Control','commented','off')
end
Waypoints = [0 0 -alt0;
             3000 0 -alt0;
             3000 3000 -alt0;
             0 3000 -alt0;
             0 0 -alt0];

set_param('gtm_design/Guidance/CAS Step','commented','on')

[tsim,xsim,ysim]=sim('gtm_design',[0 t_final]);

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
% dele_deg_2      = SurfacePos(:,1);
% dela_deg_2      = SurfacePos(:,6);
% delr_deg_2      = SurfacePos(:,7);
% delml_percent_2 = SurfacePos(:,14);
% delmr_percent_2 = SurfacePos(:,16);
% delms_percent_2 = (delml_percent+delmr_percent)/2;
% delma_percent_2 = (delml_percent-delmr_percent)/2;
delt_percent_2  = Throttle;

p_N_ft_2 = NED(:,1);
p_E_ft_2 = NED(:,2);
p_D_ft_2 = NED(:,3);

%%
fig = figure('units','centimeters','position',[13 2 13 20]);
subplot(5,1,1), plot(t_1,tas_kt_1,'b',t_2,tas_kt_2,'--r',t_2,t_2*0+tas0,'-.'), ylabel('True Airspeed [kt]'), xlabel('Time [sec]')
grid on, grid minor, hold on, legend('LPV','H_\infty','Command','location','best')
subplot(5,1,2), plot(t_1,alp_deg_1,'b',t_2,alp_deg_2,'--r'), ylabel('Angle of Attack [deg]'), xlabel('Time [sec]')
grid on, grid minor, hold on, legend('LPV','H_\infty','location','best')
subplot(5,1,3), plot(t_1,q_rad_1*r2d,'b',t_2,q_rad_2*r2d,'--r'), ylabel('Angle of Attack [deg]'), xlabel('Time [sec]')
grid on, grid minor, hold on, legend('LPV','H_\infty','location','best')
subplot(5,1,4), plot(t_1,the_rad_1*r2d,'b',t_2,the_rad_2*r2d,'--r'), ylabel('Angle of Attack [deg]'), xlabel('Time [sec]')
grid on, grid minor, hold on, legend('LPV','H_\infty','location','best')
subplot(5,1,5), plot(t_1,alt_ft_1,'b',t_2,alt_ft_2,'--r',t_2,t_2*0+alt0,'-.'), ylabel('Altitude [ft]'), xlabel('Time [sec]')
grid on, grid minor, hold on, legend('LPV','H_\infty','Command','location','best')

exportgraphics(fig,'figure/autopilot_straight_state.png','resolution',300)

%%
fig = figure('units','centimeters','position',[13*2 2 13 20]);

subplot(3,1,1), plot(SurfacePos_1.FLAPLOB.Time,(SurfacePos_1.FLAPLOB.Data+SurfacePos_1.FLAPROB.Data)/2), ylabel('Sym. Morphing [%]'), xlabel('Time [sec]')
grid on, grid minor, hold on, xlim([0 t_final])
subplot(3,1,2), plot(SurfacePos_1.AILL.Time,SurfacePos_1.ELLOB.Data,'b',SurfacePos_2.AILL.Time,SurfacePos_2.ELLOB.Data,'--r'), ylabel('Elevator [deg]'), xlabel('Time [sec]')
grid on, grid minor, hold on, legend('LPV','H_\infty','location','best'), xlim([0 t_final])
subplot(3,1,3), plot(SurfacePos_1.AILL.Time,delt_percent_1,'b',SurfacePos_2.AILL.Time,delt_percent_2,'--r'), ylabel('Throttle [%]'), xlabel('Time [sec]')
grid on, grid minor, hold on, legend('LPV','H_\infty','location','best'), xlim([0 t_final])
exportgraphics(fig,'figure/autopilot_straight_input.png','resolution',300)

clearvars fig