%% Nonlinear simulation
FlightMode = 4.1;
phi_to_eta = 12.5/45;

etas0 = -12.5; % [%]
tas0  = 75; % [kt]
deltc = MWS.bias.throttle;
ControlMode = 4;
t_final = 90;
LookaheadDistance = 700;
TransitionRadius = 1000*0.3048;
Radius = 2000; % not used
TurnDirection = 1; % not used

alt0  = 1000; % [ft]
MWS               = init_design('GTM',1);
MWS.bias.morphing = etas0; % set morphing parameter
MWS.StatesInp(9)  = alt0; % set altitude [ft]
target            = struct('gamma',0,'tas',tas0); % start trim condition target [knot]
set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','on') % control off
set_param('gtm_design/Guidance','commented','on')
set_param('gtm_design/Monitor','commented','on')
set_param('gtm_design','MinStep','auto')
MWS.SurfaceDeadbandOn = 0;
loadmws(MWS,'gtm_design'); % load MWS
[MWS,Xtrim,Trimcond,Err] = trimgtm(target,'elev',1); % trim calculation
MWS.SurfaceDeadbandOn = 1;
loadmws(MWS,'gtm_design'); % load MWS

set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','off') % control on
set_param('gtm_design/Guidance','commented','off')
set_param('gtm_design/Monitor','commented','off')
set_param('gtm_design','MaxStep','auto')
set_param('gtm_design','RelTol','1e-2')
% set_param('gtm_design','MinStep','0.0049')
set_param('gtm_design','MinStep','0.002')

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

[tsim,xsim,ysim]=sim('gtm_design',[0 t_final]);

%%

set(0,'DefaultLineLineWidth',1)

t       = tsim; % [sec]

eas_kt  = ysim(:,1); % [knot]
tas_kt  = ysim(:,2); % [knot]
alp_deg = ysim(:,3); % [deg]
bet_deg = ysim(:,4); % [deg]
gam_deg = ysim(:,5); % [deg]
chi_deg = ysim(:,6); % [deg]
u_ft    = ysim(:,7); % [ft/s]
v_ft    = ysim(:,8); % [ft/s]
w_ft    = ysim(:,9); % [ft/s]
p_rad   = ysim(:,10); % [rad/s]
q_rad   = ysim(:,11); % [rad/s]
r_rad   = ysim(:,12); % [rad/s]
lat_rad = ysim(:,13); % [rad]
lon_rad = ysim(:,14); % [rad]
alt_ft  = ysim(:,15); % [ft]
phi_rad = ysim(:,16); % [rad]
the_rad = ysim(:,17); % [rad]
psi_rad = ysim(:,18); % [rad]
Ax_g    = ysim(:,19); % [g]
Ay_g    = ysim(:,20); % [g]
Az_g    = ysim(:,21); % [g]

tas_c_kt       = command(:,1); % 
alt_c_ft       = command(:,2); % 
bet_c_deg      = command(:,3); % 
phi_c_deg      = command(:,4); % 
an_c_g         = command(:,5); % 
p_c_deg        = command(:,6); % 
etas_c_percent = command(:,7); % 

% SurfacePos = SurfacePos;
% dele_deg      = SurfacePos(:,1);
% dela_deg      = SurfacePos(:,6);
% delr_deg      = SurfacePos(:,7);
% delml_percent = SurfacePos(:,14);
% delmr_percent = SurfacePos(:,16);
% delms_percent = (delml_percent+delmr_percent)/2;
% delma_percent = (delml_percent-delmr_percent)/2;
% delt_percent  = Throttle;

p_N_ft = NED(:,1);
p_E_ft = NED(:,2);
p_D_ft = NED(:,3);

%%
fig1 = figure('units','centimeters','position',[0 2 26 20]);
trajectory3(p_E_ft,p_N_ft,-p_D_ft,-phi_rad,the_rad,-psi_rad,1/150,4000,'boeing_747',[90 0],'blue','black');
xlabel('East [ft]')
ylabel('North [ft]')
zlabel('Up [ft]')
box on
grid on
grid minor, hold on
view([45 30])
axis equal
view([45 30])
xlim([-1500 3500])
ylim([-500 3500])
plot3([0 0 3000 3000 0],[0 3000 3000 0 0],[1000 1000 1000 1000 1000],'--or')
legend('Trajectory','Waypoint','location','best')
exportgraphics(fig1,'figure/waypoint_following_trajectory.png','resolution',300)
savefig(fig1,'figure/waypoint_following_trajectory.fig')

%%
fig2 = figure('units','centimeters','position',[13 2 13 20]);

subplot(5,1,1), plot(t,tas_kt,t,t*0+75), ylabel('True Airspeed [kt]'), xlabel('Time [sec]')
grid on, grid minor, legend('V','V_c','location','best')
subplot(5,1,2), plot(t,alt_ft,t,t*0+1000), ylabel('Altitude [ft]'), xlabel('Time [sec]')
grid on, grid minor, legend('h','h_c','location','best')
subplot(5,1,3), plot(t,bet_deg,t,t*0), ylabel('Sideslip [deg]'), xlabel('Time [sec]')
grid on, grid minor, legend('\beta','\beta_c','location','best')
subplot(5,1,4), plot(t,phi_rad*r2d,t,phi_c_deg), ylabel('Roll [deg]'), xlabel('Time [sec]')
grid on, grid minor, legend('\phi','\phi_c','location','best')
subplot(5,1,5), plot(SurfacePos.FLAPLOB.Time,(SurfacePos.FLAPLOB.Data+SurfacePos.FLAPROB.Data)/2), ylabel('Sym. Morphing [%]'), xlabel('Time [sec]')
grid on, grid minor, hold on

exportgraphics(fig2,'figure/waypoint_following_command.png','resolution',300)
savefig(fig2,'figure/waypoint_following_command.fig')
