%% Nonlinear simulation
FlightMode = 3.0;
ControlMode = 3;
phi_to_eta = 0;

etas0 = -12.5; % [%]
tas0  = 75; % [kt]
deltc = MWS.bias.throttle;
t_final = 30;
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
set_param('gtm_design','MinStep','0.0049')

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
