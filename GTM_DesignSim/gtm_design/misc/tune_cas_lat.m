%% Initialization
close all
clear all
clc

% Model basic setup
setup

%%
% load lpv_design_data.mat

%% lateral-directional control design
% actuator model
P_a_ail = 2*pi*5/(s+2*pi*5); % Bandwidth 5 Hz and 5+10 ms time delay
P_a_rud = 2*pi*5/(s+2*pi*5); % Bandwidth 5 Hz and 5+10 ms time delay
P_a_mor = 2*pi*0.5/(s+2*pi*0.5); % Bandwidth 5 Hz and 5+10 ms time delay
P_a_lat_ex = blkdiag(P_a_ail,P_a_rud,P_a_mor);

% maximum expected command
W_r_bet = 3*d2r;
W_r_p   = 10*d2r;
W_r_lat_ex = blkdiag(W_r_bet,W_r_p);

% ideal model
omega_bet = 2; % [rad/s]
zeta_bet = 1; % []
W_m_bet = tf(omega_bet^2,[1 2*zeta_bet*omega_bet omega_bet^2]);
omega_p = 10; % [rad/s]
zeta_p = 1; % []
W_m_p = tf(omega_p^2,[1 2*zeta_p*omega_p omega_p^2]);
% W_m_bet = 5.78/(s^2+3.92*s+5.78);
% W_m_p = 39.06/(s^2+8.75+39.06);
W_m_lat_ex = blkdiag(W_m_bet,W_m_p);

% noise model
W_n_bet = 0.033*d2r; % 0.033 deg
W_n_p   = 0.5220*d2r; % 0.5220 deg/sec
W_n_r   = 0.3330*d2r; % 0.3330 deg/sec
W_n_lat_ex = blkdiag(W_n_bet,W_n_p,W_n_r);

% control weight
W_c_ail = (s/31.4+1)/(s/100+1)*0.01; % [deg]
W_c_rud = (s/31.4+1)/(s/100+1)*0.01; % [deg]
W_c_mor = (s/3.14+1)/(s/10+1)*0.03; % [deg]
W_c_lat_ex = blkdiag(W_c_ail,W_c_rud,W_c_mor);

% performance weight
% W_p_bet = 1/(s/0.001+1)*r2d*20;
% W_p_p   = 1/(s/0.001+1)*r2d*1;
W_p_bet = (s/59.73+1)/(s/0.299+1)/10*59.73/0.299*10;
W_p_p   = (s/59.73+1)/(s/0.001+1)/10*59.73/0.299*100;
W_p_lat_ex = blkdiag(W_p_bet,W_p_p);

% time delay
TD3 = blkdiag(TD30ms,TD30ms,TD30ms);

% lateral LPV modeling
systemnames         = 'P_lat_ex W_m_lat_ex W_n_lat_ex W_c_lat_ex W_p_lat_ex W_r_lat_ex P_a_lat_ex TD3';
inputvar            = '[r{2} ; n{3} ; u{3}]';
outputvar           = '[W_c_lat_ex ; W_p_lat_ex ; W_r_lat_ex(1)-TD3(1) ; W_r_lat_ex(2)-TD3(2) ; TD3(3)]';
input_to_P_lat_ex   = '[P_a_lat_ex]';
input_to_P_a_lat_ex = '[u]';
input_to_W_m_lat_ex = '[W_r_lat_ex]';
input_to_W_n_lat_ex = '[n]';
input_to_W_c_lat_ex = '[u]';
input_to_W_p_lat_ex = '[W_m_lat_ex(1)-P_lat_ex(1) ; W_m_lat_ex(2)-P_lat_ex(2)]';
input_to_W_r_lat_ex = '[r]';
input_to_TD3        = '[P_lat_ex+W_n_lat_ex]';
sysoutname          = 'G_lat_ex';
cleanupsysic        = 'yes'; 
sysic
num_state_G_lat_ex  = width(G_lat_ex.value('rho1_sas_param',sas_grid(1),'rho2_alt_param',alt_grid(1),'rho3_mor_param',mor_grid(1)).A);
num_input_G_lat_ex  = width(G_lat_ex.value('rho1_sas_param',sas_grid(1),'rho2_alt_param',alt_grid(1),'rho3_mor_param',mor_grid(1)).B);
num_output_G_lat_ex = height(G_lat_ex.value('rho1_sas_param',sas_grid(1),'rho2_alt_param',alt_grid(1),'rho3_mor_param',mor_grid(1)).C);
fprintf('Lateral-directional extracted generalized plant size: %ds %di %do\n',num_state_G_lat_ex,num_input_G_lat_ex,num_output_G_lat_ex)

% H-infinity synthesis
ncont_lat_ex = 3;
nmeas_lat_ex = 3;
clearvars Klatexhinf_grid gamma_lat_ex_hinf_grid
tic
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor 
    G_lat_ex_point = G_lat_ex.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [~,~,gamma_lat_ex_hinf_point_optimal]        = hinfsyn(G_lat_ex_point,nmeas_lat_ex,ncont_lat_ex);
    [Klatexhinf_point,~,gamma_lat_ex_hinf_point] = hinfsyn(G_lat_ex_point,nmeas_lat_ex,ncont_lat_ex,1.2*gamma_lat_ex_hinf_point_optimal);
    fprintf('lat Ex Hinf gamma = %.5g (%.3g%% back-off from %.5g)\n',gamma_lat_ex_hinf_point,(gamma_lat_ex_hinf_point/gamma_lat_ex_hinf_point_optimal-1)*100,gamma_lat_ex_hinf_point_optimal)
    Klatexhinf_grid(:,:,i,j,k) = Klatexhinf_point;
    gamma_lat_ex_hinf(i,j,k) = gamma_lat_ex_hinf_point;
end
end
end
time_lat_ex_hinf = toc;
Klatexhinf = pss(Klatexhinf_grid,Domain);
save('data/lat_ex_hinf.mat','Klatexhinf','gamma_lat_ex_hinf','time_lat_ex_hinf')

% LPV synthesis
tic
[Klatexlpv,gamma_lat_ex_lpv,info_lat_ex_lpv] = lpvsyn(G_lat_ex,nmeas_lat_ex,ncont_lat_ex);
time_lat_ex_lpv = toc;
save('data/lat_ex_lpv.mat','Klatexlpv','gamma_lat_ex_lpv','info_lat_ex_lpv','time_lat_ex_lpv')
fprintf('lat Ex LPV  gamma = %.5g (%.3g%% back-off from %.5g)\n',gamma_lat_ex_lpv,(gamma_lat_ex_lpv/info_lat_ex_lpv.MinGamma-1)*100,info_lat_ex_lpv.MinGamma)

% closed-loop interconnection with lateral-directional H_infinity controller
Platexscl = diag([r2d r2d r2d]); % [deg] x [deg] x [deg]
Klatexscl = diag([1 1 1]); % [deg] x [deg] x [%]
systemnames         = 'P_lat_ex Klatexhinf W_r_lat_ex P_a_lat_ex TD3 Platexscl Klatexscl';
inputvar            = '[r{2}]';
outputvar           = '[Platexscl ; Klatexscl]';
input_to_P_lat_ex   = '[P_a_lat_ex]';
input_to_Klatexhinf = '[W_r_lat_ex(1)-TD3(1) ; W_r_lat_ex(2)-TD3(2) ; TD3(3)]';
input_to_W_r_lat_ex = '[r]';
input_to_P_a_lat_ex = '[Klatexhinf]';
input_to_TD3        = '[P_lat_ex]';
input_to_Platexscl  = '[P_lat_ex]';
input_to_Klatexscl  = '[Klatexhinf]';
sysoutname          = 'CL_lat_ex_hinf';
cleanupsysic        = 'yes'; 
sysic

% closed-loop interconnection with lateral-directional LPV controller
systemnames         = 'P_lat_ex Klatexlpv W_r_lat_ex P_a_lat_ex TD3 Platexscl Klatexscl';
inputvar            = '[r{2}]';
outputvar           = '[Platexscl ; Klatexscl]';
input_to_P_lat_ex   = '[P_a_lat_ex]';
input_to_Klatexlpv  = '[W_r_lat_ex(1)-TD3(1) ; W_r_lat_ex(2)-TD3(2) ; TD3(3)]';
input_to_W_r_lat_ex = '[r]';
input_to_P_a_lat_ex = '[Klatexlpv]';
input_to_TD3        = '[P_lat_ex]';
input_to_Platexscl  = '[P_lat_ex]';
input_to_Klatexscl  = '[Klatexlpv]';
sysoutname          = 'CL_lat_ex_lpv';
cleanupsysic        = 'yes'; 
sysic

% plot_lat_ex_step_lin
% plot_lat_ex_bode

%% Nonlinear simulation
etas0 = -25; % [%]
alt0  = 1000; % [ft]
% sas0  = 80; % [kt]
% tas0  = sas2tas(sas0,alt0); % [kt]
tas0  = 100; % [kt]
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

FlightMode = 2.2;
ControlMode = 2;
% deltc = MWS.bias.throttle;
deltc = 100;
% deltc = 50;
% deltc = 30;
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

[tsim,xsim,ysim]=sim('gtm_design',[0 20]);
