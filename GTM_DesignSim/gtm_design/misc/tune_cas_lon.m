%% Initialization
close all
clear all
clc

% Model basic setup
setup

%%
load lpv_design_data.mat

%% longitudinal control design
s = tf('s'); % Laplace variable

% actuator model
P_a_ele = 2*pi*5/(s+2*pi*5); % elevator model
P_a_lon_ex = blkdiag(P_a_ele); % actuator model

% reference scale (maximum expected command)
W_r_an  = 2*g; % an reference scale [ft/s^2]
W_r_lon_ex = blkdiag(W_r_an);

% ideal response
% W_m_an  = 1/(s/1+1); % ideal an model (1st-order system)
W_m_an  = 16/(s^2+5.6*s+16); % ideal an model (1st-order system)
W_m_lon_ex = blkdiag(W_m_an); % ideal response

% sensor measurement noise
W_n_alp = 0.031*d2r; % 0.031 deg alp white noise (ADS)
W_n_q   = 0.3857*d2r; % 0.3857 deg/s q white noise (MAG3)
W_n_an  = 0.0082*g; % 0.0082g white noise (MAG3)
W_n_lon_ex = blkdiag(W_n_alp,W_n_q,W_n_an);

% control weight
W_c_ele = (s/31.4+1)/(s/120+1)*0.01; % elevator weight [deg]
W_c_lon_ex = blkdiag(W_c_ele);

% performance weight
% W_p_an = (1)/(s/0.001+1)/g*100; % an tracking error weight
W_p_an = (s/59.73+1)/(s/1.76+1)/10*59.73/0.176; % an tracking error weight
W_p_lon_ex = blkdiag(W_p_an);

% time delay
TD30ms = pade(exp(-30e-3*s),1);
TD3 = blkdiag(TD30ms,TD30ms,TD30ms);

% control interconnection
systemnames         = 'P_lon_ex W_m_lon_ex W_n_lon_ex W_c_lon_ex W_p_lon_ex W_r_lon_ex P_a_lon_ex TD3';
inputvar            = '[r{1} ; n{3} ; u{1}]';
outputvar           = '[W_c_lon_ex ; W_p_lon_ex ; W_r_lon_ex+TD3(3) ; TD3(1) ; TD3(2)]';
input_to_P_lon_ex   = '[P_a_lon_ex]';
input_to_W_m_lon_ex = '[W_r_lon_ex]';
input_to_W_n_lon_ex = '[n]';
input_to_W_c_lon_ex = '[u]';
input_to_W_p_lon_ex = '[W_m_lon_ex+P_lon_ex(3)]';
input_to_W_r_lon_ex = '[r]';
input_to_P_a_lon_ex = '[u]';
input_to_TD3        = '[P_lon_ex+W_n_lon_ex]';
sysoutname          = 'G_lon_ex';
cleanupsysic        = 'yes';
sysic
num_state_G_lon_ex  = width(G_lon_ex.value('rho1_sas_param',sas_grid(1),'rho2_alt_param',alt_grid(1),'rho3_mor_param',mor_grid(1)).A);
num_input_G_lon_ex  = width(G_lon_ex.value('rho1_sas_param',sas_grid(1),'rho2_alt_param',alt_grid(1),'rho3_mor_param',mor_grid(1)).B);
num_output_G_lon_ex = height(G_lon_ex.value('rho1_sas_param',sas_grid(1),'rho2_alt_param',alt_grid(1),'rho3_mor_param',mor_grid(1)).C);
fprintf('longitudinal extracted generalized plant size: %ds %di %do\n',num_state_G_lon_ex,num_input_G_lon_ex,num_output_G_lon_ex)

% H-infinity synthesis
ncont_lon_ex = 1;
nmeas_lon_ex = 3;
clearvars Klonexhinf_grid gamma_lon_ex_hinf_grid
tic
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor 
    G_lon_ex_point = G_lon_ex.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [~,~,gamma_lon_ex_hinf_point_optimal]        = hinfsyn(G_lon_ex_point,nmeas_lon_ex,ncont_lon_ex);
    [Klonexhinf_point,~,gamma_lon_ex_hinf_point] = hinfsyn(G_lon_ex_point,nmeas_lon_ex,ncont_lon_ex,1.2*gamma_lon_ex_hinf_point_optimal);
    fprintf('lon Ex Hinf gamma = %.5g (%.3g%% back-off from %.5g)\n',gamma_lon_ex_hinf_point,(gamma_lon_ex_hinf_point/gamma_lon_ex_hinf_point_optimal-1)*100,gamma_lon_ex_hinf_point_optimal)
    Klonexhinf_grid(:,:,i,j,k) = Klonexhinf_point;
    gamma_lon_ex_hinf(i,j,k) = gamma_lon_ex_hinf_point;
end
end
end
time_lon_ex_hinf = toc;
Klonexhinf = pss(Klonexhinf_grid,Domain);
save('data/lon_ex_hinf.mat','Klonexhinf','gamma_lon_ex_hinf','time_lon_ex_hinf')

% LPV synthesis
tic
[Klonexlpv,gamma_lon_ex_lpv,info_lon_ex_lpv] = lpvsyn(G_lon_ex,nmeas_lon_ex,ncont_lon_ex);
time_lon_ex_lpv = toc;
save('data/lon_ex_lpv.mat','Klonexlpv','gamma_lon_ex_lpv','info_lon_ex_lpv','time_lon_ex_lpv')
fprintf('Lon Ex LPV  gamma = %.5g (%.3g%% back-off from %.5g)\n',gamma_lon_ex_lpv,(gamma_lon_ex_lpv/info_lon_ex_lpv.MinGamma-1)*100,info_lon_ex_lpv.MinGamma)

% closed-loop interconnection with Longitudinal H_infinity controller
Plonexscl = diag([r2d r2d 1/g]); % [deg] x [deg] x [g]
Klonexscl = diag([1]); % [deg]
systemnames         = 'P_lon_ex Klonexhinf W_r_lon_ex P_a_lon_ex TD3 Plonexscl Klonexscl';
inputvar            = '[r{1}]';
outputvar           = '[Plonexscl ; Klonexscl]';
input_to_P_lon_ex   = '[P_a_lon_ex]';
input_to_Klonexhinf = '[W_r_lon_ex+P_lon_ex(3) ; P_lon_ex(1) ; P_lon_ex(2)]';
input_to_W_r_lon_ex = '[r]';
input_to_P_a_lon_ex = '[Klonexhinf]';
input_to_TD3        = '[P_lon_ex]';
input_to_Plonexscl  = '[P_lon_ex]';
input_to_Klonexscl  = '[Klonexhinf]';
sysoutname          = 'CL_lon_ex_hinf';
cleanupsysic        = 'yes'; 
sysic

% closed-loop interconnection with Longitudinal LPV controller
systemnames         = 'P_lon_ex Klonexlpv W_r_lon_ex P_a_lon_ex TD3 Plonexscl Klonexscl';
inputvar            = '[r{1}]';
outputvar           = '[Plonexscl ; Klonexscl]';
input_to_P_lon_ex   = '[P_a_lon_ex]';
input_to_Klonexlpv  = '[W_r_lon_ex+P_lon_ex(3) ; P_lon_ex(1) ; P_lon_ex(2)]';
input_to_W_r_lon_ex = '[r]';
input_to_P_a_lon_ex = '[Klonexlpv]';
input_to_TD3        = '[P_lon_ex]';
input_to_Plonexscl  = '[P_lon_ex]';
input_to_Klonexscl  = '[Klonexlpv]';
sysoutname          = 'CL_lon_ex_lpv';
cleanupsysic        = 'yes'; 
sysic

% plot_lon_ex_step_lin
% plot_lon_ex_bode

%% Nonlinear simulation
etas0 = -12.5; % [%]
alt0  = 1000; % [ft]
% sas0  = 80; % [kt]
% tas0  = sas2tas(sas0,alt0); % [kt]
tas0  = 50; % [kt]
gam0  = 0; % [deg]

MWS               = init_design('GTM',1);
MWS.bias.morphing = etas0; % set morphing parameter
MWS.StatesInp(9)  = alt0; % set altitude [ft]
target            = struct('gamma',gam0,'tas',tas0); % start trim condition target [knot]
% MWS.bias.aileron = 0;
% MWS.bias.rudder = 0;
loadmws(MWS,'gtm_design'); % load MWS
set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','on') % control off
set_param('gtm_design/Guidance','commented','on')
set_param('gtm_design/Monitor','commented','on')
set_param('gtm_design','MinStep','auto')
[MWS,Xtrim,Trimcond,Err] = trimgtm(target,'elev',1); % trim calculation
% MWS.StatesInp(4) = 1*d2r; % p [rad/s]
% MWS.StatesInp(5) = 1*d2r; % q [rad/s]
% MWS.StatesInp(6) = 1*d2r; % r [rad/s]
% MWS.StatesInp(10) = 1*d2r; % phi [rad/s]
% MWS.StatesInp(11) = 1*d2r; % theta [rad/s]
% MWS.StatesInp(12) = 1*d2r; % psi [rad/s] 
loadmws(MWS,'gtm_design'); % load MWS

set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','off') % control on
set_param('gtm_design/Guidance','commented','off')
set_param('gtm_design/Monitor','commented','off')
set_param('gtm_design','MaxStep','auto')
set_param('gtm_design','RelTol','1e-3')
set_param('gtm_design','MinStep',num2str(MWS.Timestep-0.00001))

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

FlightMode = 2.1;
ControlMode = 2;
% deltc = MWS.bias.throttle;
deltc = 90;
% deltc = 50;
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

[tsim,xsim,ysim]=sim('gtm_design',[0 10]);
