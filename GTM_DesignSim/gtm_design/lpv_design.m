%% initialization
close all; clear all; clc
set(0,'DefaultLineLineWidth',1)

% simulink model setup
setup

%% Domain definition
% unit conversion parameters
knots2fps = 1.68781; % knots to feet per seconds
fps2knots = 1/knots2fps; % knots to feet per seconds
m2ft      = 3.28084; % meters to feet
d2r       = pi/180; % degrees to radians
r2d       = 180/pi;
g = 32.174; % gravitational acceleration [ft/s^2]
% Coeff_throttle = [0.000033035538475  -0.008354385314499 1.191126728328792 31.518959847101669];
% Coeff_thrust   = [0.000014847593013,  0, 0, 0.412662356277121];
% T_max = polyval(Coeff_thrust,polyval(Coeff_throttle,100)); % Maximum thrust in [lbf] at sea level without ram drag

% scheduling parameter gridding
sas_grid  = [60 70 80 90]; % [kt]
alt_grid  = [0 5000 10000]; % 700~1100 [ft]
mor_grid  = [-25 -12.5 0]; % between -25 and 0 [%]
num_sas   = numel(sas_grid); % number sas grid
num_alt   = numel(alt_grid); % number alt grid
num_mor   = numel(mor_grid); % number morphing grid

% parameters for rate-bounded design
sas_RateBound = 5; % 2*T_max/MWS.w0 = 0.53g = 17 ft/s^2
alt_RateBound = 5; % convvel([100],'kts','ft/s')
mor_RateBound = 5; % 10 sec for full deployment

% construct Domain 
Domain = rgrid({'rho1_sas_param','rho2_alt_param','rho3_mor_param'},{sas_grid,alt_grid,mor_grid});
Domain_RB = rgrid({'rho1_sas_param','rho2_alt_param','rho3_mor_param'},{sas_grid,alt_grid,mor_grid},[[-1,1]*sas_RateBound;[-1,1]*alt_RateBound;[-1,1]*mor_RateBound]);

% plot_parameter_grid

%% trimming
set_param('gtm_design','MinStep','auto')
set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','on') % switch off controller

clearvars MWS_grid sas_trim tas_trim alt_trim mor_trim alp_trim pitch_trim throttle_trim aileron_trim elevator_trim rudder_trim
clearvars sys_sas_trim sys_tas_trim sys_alt_trim sys_mor_trim sys_alp_trim sys_pitch_trim sys_throttle_trim sys_aileron_trim sys_elevator_trim sys_rudder_trim
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    sas = sas_grid(i); % trim sas [kt]
    alt = alt_grid(j); % trim alt [ft]
    mor = mor_grid(k); % trim mor [%]
    tas = sas2tas(sas,alt); % [kts]

    target = struct('tas',tas,'gamma',0); % target trim condition [kt] x [deg]
    MWS = grabmws('gtm_design');
    MWS.StatesInp(9)  = alt; % set trim altitude [ft]
    MWS.bias.morphing = mor; % set trim symmetric morphing parameter [%]
    loadmws(MWS,'gtm_design');

    [MWS,Xtrim,Trimcond,Err] = trimgtm(target,'elev',0); % trim calculation (attempt #1)
    if (Err > 1e-3)
        loadmws(init_design(),'gtm_design');
        warning('Poor convergence, trying again.\n');
        [MWS,Xtrim,Trimcond,Err] = trimgtm(target,'elev',0); % trim calculation (attempt #2)
        if (Err > 1e-3)
            loadmws(init_design(),'gtm_design');
            warning('Poor convergence, trying again.\n');
            [MWS,Xtrim,Trimcond,Err] = trimgtm(target,'elev',0); % trim calculation (attempt #3)
            if (Err > 1e-3)
                error('TRIM FAIL')
            end
        end
    end
    MWS_grid(i,j,k) = MWS;
    fprintf('trimming #%d @ tas: %d kt, alt: %d ft, mor: %.3g %%\n',sub2ind([num_mor num_alt num_sas],k,j,i),tas,alt,mor)

    % extract trim condition
    sas_trim(i,j,k)      = sas; % [kts]
    tas_trim(i,j,k)      = tas; % [kts]
    alt_trim(i,j,k)      = alt; % [ft]
    mor_trim(i,j,k)      = mor; % [%]
    alp_trim(i,j,k)      = Trimcond.alpha; % [deg]
    pitch_trim(i,j,k)    = Trimcond.pitch; % [deg]
    throttle_trim(i,j,k) = Trimcond.throttle; % [%]
    aileron_trim(i,j,k)  = Trimcond.aileron; % [deg]
    elevator_trim(i,j,k) = Trimcond.elevator; % [deg]
    rudder_trim(i,j,k)   = Trimcond.rudder; % [deg]
    
    % state-space model (y=u)
    sys_sas_trim(:,:,i,j,k)      = ss(0,0,0,sas_trim(i,j,k)); % [kts]
    sys_tas_trim(:,:,i,j,k)      = ss(0,0,0,tas_trim(i,j,k)); % [kts]
    sys_alt_trim(:,:,i,j,k)      = ss(0,0,0,alt_trim(i,j,k)); % [ft]
    sys_mor_trim(:,:,i,j,k)      = ss(0,0,0,mor_trim(i,j,k)); % []
    sys_alp_trim(:,:,i,j,k)      = ss(0,0,0,alp_trim(i,j,k)); % [deg]
    sys_pitch_trim(:,:,i,j,k)    = ss(0,0,0,pitch_trim(i,j,k)); % [deg]
    sys_throttle_trim(:,:,i,j,k) = ss(0,0,0,throttle_trim(i,j,k)); % [%]
    sys_aileron_trim(:,:,i,j,k)  = ss(0,0,0,aileron_trim(i,j,k)); % [deg]
    sys_elevator_trim(:,:,i,j,k) = ss(0,0,0,elevator_trim(i,j,k)); % [deg]
    sys_rudder_trim(:,:,i,j,k)   = ss(0,0,0,rudder_trim(i,j,k)); % [deg]
end
end
end

% trim condition scheduling
sch_sas_trim      = pss(sys_sas_trim,Domain); % [deg]
sch_tas_trim      = pss(sys_tas_trim,Domain); % [deg]
sch_alt_trim      = pss(sys_alt_trim,Domain); % [deg]
sch_mor_trim      = pss(sys_mor_trim,Domain); % [deg]
sch_alp_trim      = pss(sys_alp_trim,Domain); % [deg]
sch_pitch_trim    = pss(sys_pitch_trim,Domain); % [deg]
sch_throttle_trim = pss(sys_throttle_trim,Domain); % [%]
sch_aileron_trim  = pss(sys_aileron_trim,Domain); % [deg]
sch_elevator_trim = pss(sys_elevator_trim,Domain); % [deg]
sch_rudder_trim   = pss(sys_rudder_trim,Domain); % [deg]

% struct scheduled trim condition
sch_param.sch_sas_trim      = sch_sas_trim;
sch_param.sch_tas_trim      = sch_tas_trim;
sch_param.sch_alt_trim      = sch_alt_trim;
sch_param.sch_mor_trim      = sch_mor_trim;
sch_param.sch_alp_trim      = sch_alp_trim;
sch_param.sch_pitch_trim    = sch_pitch_trim;
sch_param.sch_throttle_trim = sch_throttle_trim;
sch_param.sch_aileron_trim  = sch_aileron_trim;
sch_param.sch_elevator_trim = sch_elevator_trim;
sch_param.sch_rudder_trim   = sch_rudder_trim;

% save scheduled trim condition
save('data/trim_data.mat','sch_param')

% plot_trim_condition

%% linearization
set_param('gtm_design','MinStep','auto')
set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','on') % switch off controller

clearvars GTM_lon GTM_lat GTM_lon_ex GTM_lat_ex
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor 
    sas = sas_grid(i); % trim sas [kt]
    alt = alt_grid(j); % trim alt [ft]
    mor = mor_grid(k); % trim mor [%]
    tas = sas2tas(sas,alt); % [kt]

    % linearization
    [sys,londyn,latdyn]=linmodel(MWS_grid(i,j,k),1,0,0);
    fprintf('Linearization #%d @ TAS: %d kt, Alt: %d ft, Mor: %.3g %%\n',sub2ind([num_mor num_alt num_sas],k,j,i),tas,alt,mor)

    % stack LTI model
    LTI_lon(:,:,i,j,k) = londyn;
    LTI_lat(:,:,i,j,k) = latdyn;

    % input name
    set(londyn,'Statename',{'tas_ft','alp_rad','q_rad','theta_rad'},...
               'Inputname',{'dele_deg','morl_percent','morr_percent','deltl_percent','deltr_percent'},...
               'Outputname',{'tas_ft','alp_rad','q_rad','theta_rad'});
    set(latdyn,'Statename',{'beta_rad','p_rad','r_rad','phi_rad'},...
               'Inputname',{'dela_deg','delr_deg','morl_percent','morr_percent'},...
               'Outputname',{'beta_rad','p_rad','r_rad','phi_rad'});
    
    % augment altitude dynamics & modify input
    Alon_aug = [londyn.A zeros(4,1); 0 -tas*knots2fps 0 tas*knots2fps 0]; % altitude [ft]
    Blon_aug = [londyn.B(:,4)+londyn.B(:,5) londyn.B(:,1); 0 0];
    Clon_aug = eye(5);
    Dlon_aug = 0;

    % modify input
    Alat_aug = latdyn.A;
    Blat_aug = [latdyn.B(:,1:2) (latdyn.B(:,4)-latdyn.B(:,3))/2]; % asymmetric morphing parameter [percent]
    Clat_aug = eye(4);
    Dlat_aug = 0;
    
    % Form state-space model
    londyn_aug = ss(Alon_aug,Blon_aug,Clon_aug,Dlon_aug);
    latdyn_aug = ss(Alat_aug,Blat_aug,Clat_aug,Dlat_aug);
    set(londyn_aug,'Statename',{'tas_ft','alp_rad','q_rad','theta_rad','alt_ft'},...
                   'Inputname',{'delt_percent','dele_deg'},...
                   'Outputname',{'tas_ft','alp_rad','q_rad','theta_rad','alt_ft'});
    set(latdyn_aug,'Statename',{'beta_rad','p_rad','r_rad','phi_rad'},...
                   'Inputname',{'dela_deg','delr_deg','mor_percent'},...
                   'Outputname',{'beta_rad','p_rad','r_rad','phi_rad'});

    % stack LTI model
    GTM_lon(:,:,i,j,k) = londyn_aug;
    GTM_lat(:,:,i,j,k) = latdyn_aug;

    % extract short-period dynamics & modify input
    Z_alp = londyn.A(2,2)*tas*knots2fps;
    Z_ele = londyn.B(2,1)*tas*knots2fps;
    Alon_ex = londyn.A(2:3,2:3);
    Blon_ex = londyn.B(2:3,1);
    Clon_ex = [eye(2); Z_alp 0]; % z acceleration [ft/s^2], 1g = 32.174 ft/s^2
    Dlon_ex = [0; 0; Z_ele];

    % modify input
    Alat_ex = latdyn.A(1:3,1:3); % eliminate roll angle dynamics
    Blat_ex = [latdyn.B(1:3,1:2) (latdyn.B(1:3,4)-latdyn.B(1:3,3))/2]; % asymmetric morphing parameter [percent]
    Clat_ex = eye(3);
    Dlat_ex = 0;
    
    % Form state-space model
    londyn_ex = ss(Alon_ex,Blon_ex,Clon_ex,Dlon_ex);
    latdyn_ex = ss(Alat_ex,Blat_ex,Clat_ex,Dlat_ex);
    set(londyn_ex,'Statename',{'alp_rad','q_rad'},...
                   'Inputname',{'dele_deg'},...
                   'Outputname',{'alp_rad','q_rad','az_ft'});
    set(latdyn_ex,'Statename',{'beta_rad','p_rad','r_rad'},...
                   'Inputname',{'dela_deg','delr_deg','mor_percent'},...
                   'Outputname',{'beta_rad','p_rad','r_rad'});

    % stack LTI model
    GTM_lon_ex(:,:,i,j,k) = londyn_ex;
    GTM_lat_ex(:,:,i,j,k) = latdyn_ex;
end
end
end

% construct LPV model
P_lon = pss(GTM_lon,Domain);
P_lat = pss(GTM_lat,Domain);
P_lon_ex = pss(GTM_lon_ex,Domain);
P_lat_ex = pss(GTM_lat_ex,Domain);

% save LPV model
save('data/linear_model_data.mat','P_lon','P_lat','P_lon_ex','P_lat_ex')

% plot_mode

save('data/trim_and_linearize.mat')

%% longitudinal control design
s = tf('s'); % Laplace variable

% actuator model
A_eng = [-1.336,-0.7314;1,0];
B_eng = [1;0];
C_eng = [-0.1474   0.7314];
D_eng = 0;
P_a_eng = tf(ss(A_eng,B_eng,C_eng,D_eng)); % throttle lag model
P_a_ele = 2*pi*5/(s+2*pi*5); % elevator model
P_a_lon = blkdiag(P_a_eng,P_a_ele); % actuator model
% figure('units','centimeters','position',[0 2 13 10])
% step(P_a_eng,P_a_ele)
% grid on
% legend('$Pa\_t$','$Pa\_e$','location','best','interpreter','latex')

% reference scale (maximum expected command)
W_r_tas = 10*knots2fps; % tas reference scale [ft/s]
W_r_alt = 10; % altitude reference scale [ft]
W_r_lon = blkdiag(W_r_tas,W_r_alt);

% ideal response
omega_tas = 2; % ideal tas model natural frequency [rad/s]
omega_alt = 2; % ideal alt model natural frequency [rad/s]
% omega_tas = .2; % ideal tas model natural frequency [rad/s]
% omega_alt = .4; % ideal alt model natural frequency [rad/s]
zeta_tas = 1; % ideal tas model damping ratio []
zeta_alt = 1; % ideal alt model damping ratio []
W_m_tas = tf(omega_tas^2,[1 2*zeta_tas*omega_tas omega_tas^2]); % ideal tas model (2nd-order system)
W_m_alt = tf(omega_alt^2,[1 2*zeta_alt*omega_alt omega_alt^2]); % ideal alt model (2nd-order system)
W_m_lon = blkdiag(W_m_tas,W_m_alt); % ideal response
% figure('units','centimeters','position',[13 2 13 10])
% step(W_m_tas,W_m_alt)
% grid on
% legend('$Wm\_V$','$Wm\_h$','location','best','interpreter','latex')

% sensor measurement noise
W_n_tas   = 0.06*knots2fps; % 0.06 kt tas white noise (ADS)
W_n_alp   = 0.031*d2r; % 0.031 deg alp white noise (ADS)
W_n_q     = 0.3857*d2r; % 0.3857 deg/s q white noise (MAG3)
W_n_theta = 0.02*d2r; % 0.02 deg theta white noise (MIDG II)
W_n_alt   = 0.1; % altitude white noise [ft] (MIDG II GPS)
W_n_lon = blkdiag(W_n_tas,W_n_alp,W_n_q,W_n_theta,W_n_alt);

% control weight
W_c_eng = (s/0.8552+1)/(s/2+1)*0.001; % throttle weight [%]
W_c_ele = (s/31.4+1)/(s/120+1)*0.01; % elevator weight [deg]
W_c_lon = blkdiag(W_c_eng,W_c_ele);
% figure('units','centimeters','position',[26 2 13 10])
% bodemag(W_c_eng,W_c_ele)
% grid on
% legend('$Wc\_t$','$Wc\_e$','location','best','interpreter','latex')

% performance weight
% W_p_tas = (s/100+1)/(s/0.005+1)/(knots2fps*10)*100; % tas tracking error weight
% W_p_alt = (s/100+1)/(s/0.005+1)/(knots2fps*10)*10; % alt tracking error weight
W_p_tas = 1/(s/0.001+1)*fps2knots*10; % tas tracking error weight
W_p_alt = 1/(s/0.001+1)*10; % alt tracking error weight
W_p_lon = blkdiag(W_p_tas,W_p_alt);
% figure('units','centimeters','position',[39 2 13 10])
% bodemag(W_p_tas,W_p_alt)
% grid on
% legend('$Wp\_V$','$Wp\_h$','location','best','interpreter','latex')

% time delay
TD30ms = pade(exp(-30e-3*s),1);
TD70ms = pade(exp(-70e-3*s),1);
TD5 = blkdiag(TD30ms,TD30ms,TD30ms,TD70ms,TD70ms);

% control interconnection
systemnames         = 'P_lon W_m_lon W_n_lon W_c_lon W_p_lon W_r_lon P_a_lon TD5';
inputvar            = '[r{2} ; n{5} ; u{2}]';
outputvar           = '[W_c_lon ; W_p_lon ; W_r_lon(1)-TD5(1) ; W_r_lon(2)-TD5(5) ; TD5(2) ; TD5(3) ; TD5(4)]';
input_to_P_lon      = '[P_a_lon]';
input_to_W_m_lon    = '[W_r_lon]';
input_to_W_n_lon    = '[n]';
input_to_W_c_lon    = '[u]';
input_to_W_p_lon    = '[W_m_lon(1)-P_lon(1) ; W_m_lon(2)-P_lon(5)]';
input_to_W_r_lon    = '[r]';
input_to_P_a_lon    = '[u]';
input_to_TD5        = '[P_lon+W_n_lon]';
sysoutname          = 'G_lon';
cleanupsysic        = 'yes';
sysic
num_state_G_lon  = width(G_lon.value('rho1_sas_param',sas_grid(1),'rho2_alt_param',alt_grid(1),'rho3_mor_param',mor_grid(1)).A);
num_input_G_lon  = width(G_lon.value('rho1_sas_param',sas_grid(1),'rho2_alt_param',alt_grid(1),'rho3_mor_param',mor_grid(1)).B);
num_output_G_lon = height(G_lon.value('rho1_sas_param',sas_grid(1),'rho2_alt_param',alt_grid(1),'rho3_mor_param',mor_grid(1)).C);
fprintf('longitudinal generalized plant size: %ds %di %do\n',num_state_G_lon,num_input_G_lon,num_output_G_lon)

% H-infinity synthesis
ncont_lon = 2;
nmeas_lon = 5;
clearvars K_lon_hinf_grid gamma_lon_hinf_grid
tic
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor 
    G_lon_point = G_lon.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [~,~,gamma_lon_hinf_point_optimal]        = hinfsyn(G_lon_point,nmeas_lon,ncont_lon);
    [K_lon_hinf_point,~,gamma_lon_hinf_point] = hinfsyn(G_lon_point,nmeas_lon,ncont_lon,1.2*gamma_lon_hinf_point_optimal);
    fprintf('lon Hinf gamma = %.5g (%.3g%% back-off from %.5g)\n',gamma_lon_hinf_point,(gamma_lon_hinf_point/gamma_lon_hinf_point_optimal-1)*100,gamma_lon_hinf_point_optimal)
    K_lon_hinf_grid(:,:,i,j,k) = K_lon_hinf_point;
    gamma_lon_hinf(i,j,k) = gamma_lon_hinf_point;
end
end
end
time_lon_hinf = toc;
K_lon_hinf = pss(K_lon_hinf_grid,Domain);
save('data/lon_hinf.mat','K_lon_hinf','gamma_lon_hinf','time_lon_hinf')

% LPV synthesis
tic
[K_lon_lpv,gamma_lon_lpv,info_lon_lpv] = lpvsyn(G_lon,nmeas_lon,ncont_lon);
time_lon_lpv = toc;
save('data/lon_lpv.mat','K_lon_lpv','gamma_lon_lpv','info_lon_lpv','time_lon_lpv')
fprintf('Lon LPV  gamma = %.5g (%.3g%% back-off from %.5g)\n',gamma_lon_lpv,(gamma_lon_lpv/info_lon_lpv.MinGamma-1)*100,info_lon_lpv.MinGamma)

% closed-loop interconnection with Longitudinal H_infinity controller
P_lon_scl = diag([fps2knots r2d r2d r2d 1]); % [kt] x [deg] x [deg] x [deg] x [ft]
K_lon_scl = diag([1 1]); % [%] x [deg]
systemnames         = 'P_lon K_lon_hinf W_r_lon P_a_lon TD5 P_lon_scl K_lon_scl';
inputvar            = '[r{2}]';
outputvar           = '[P_lon_scl ; K_lon_scl]';
input_to_P_lon      = '[P_a_lon]';
input_to_K_lon_hinf = '[W_r_lon(1)-TD5(1) ; W_r_lon(2)-TD5(5) ; TD5(2) ; TD5(3) ; TD5(4)]';
input_to_W_r_lon    = '[r]';
input_to_P_a_lon    = '[K_lon_hinf]';
input_to_TD5        = '[P_lon]';
input_to_P_lon_scl  = '[P_lon]';
input_to_K_lon_scl  = '[K_lon_hinf]';
sysoutname          = 'CL_lon_hinf';
cleanupsysic        = 'yes'; 
sysic

% closed-loop interconnection with Longitudinal LPV controller
systemnames         = 'P_lon K_lon_lpv W_r_lon P_a_lon TD5 P_lon_scl K_lon_scl';
inputvar            = '[r{2}]';
outputvar           = '[P_lon_scl ; K_lon_scl]';
input_to_P_lon      = '[P_a_lon]';
input_to_K_lon_lpv  = '[W_r_lon(1)-TD5(1) ; W_r_lon(2)-TD5(5) ; TD5(2) ; TD5(3) ; TD5(4)]';
input_to_W_r_lon    = '[r]';
input_to_P_a_lon    = '[K_lon_lpv]';
input_to_TD5        = '[P_lon]';
input_to_P_lon_scl  = '[P_lon]';
input_to_K_lon_scl  = '[K_lon_lpv]';
sysoutname          = 'CL_lon_lpv';
cleanupsysic        = 'yes'; 
sysic

% plot_lon_step_lin
% plot_lon_bode

%% lateral-directional control design
% actuator model
P_a_ail = 2*pi*5/(s+2*pi*5); % Bandwidth 5 Hz and 5+10 ms time delay
P_a_rud = 2*pi*5/(s+2*pi*5); % Bandwidth 5 Hz and 5+10 ms time delay
P_a_mor = 2*pi*0.5/(s+2*pi*0.5); % Bandwidth 5 Hz and 5+10 ms time delay
P_a_lat = blkdiag(P_a_ail,P_a_rud,P_a_mor);
% figure('units','centimeters','position',[0 2 13 10])
% step(P_a_ail,P_a_rud,P_a_mor)
% grid on
% legend('$Pa\_a$','$Pa\_r$','$Pa\_m$','location','best','interpreter','latex')

% maximum expected command
W_r_bet = 3*d2r;
W_r_phi = 45*d2r;
W_r_lat = blkdiag(W_r_bet,W_r_phi);

% ideal model
omega_bet = 2; % [rad/s]
zeta_bet = 1; % []
W_m_bet = tf(omega_bet^2,[1 2*zeta_bet*omega_bet omega_bet^2]);
omega_phi = 6; % [rad/s]
zeta_phi = 1; % []
W_m_phi = tf(omega_phi^2,[1 2*zeta_phi*omega_phi omega_phi^2]);
W_m_lat = blkdiag(W_m_bet,W_m_phi);
% figure('units','centimeters','position',[13 2 13 10])
% step(W_m_bet,W_m_phi)
% grid on
% legend('$Wm\_\beta$','$Wm\_\phi$','location','best','interpreter','latex')

% noise model
W_n_bet = 0.033*d2r; % 0.033 deg
W_n_p   = 0.5220*d2r; % 0.5220 deg/sec
W_n_r   = 0.3330*d2r; % 0.3330 deg/sec
W_n_phi = 0.02*d2r; % 0.02 deg
W_n_lat = blkdiag(W_n_bet,W_n_p,W_n_r,W_n_phi);

% control weight
W_c_ail = (s/31.4+1)/(s/100+1)*0.01; % [deg]
W_c_rud = (s/31.4+1)/(s/100+1)*0.01; % [deg]
W_c_mor = (s/3.14+1)/(s/10+1)*0.03; % [deg]
W_c_lat = blkdiag(W_c_ail,W_c_rud,W_c_mor);
% figure('units','centimeters','position',[26 2 13 10])
% bode(W_c_ail)
% hold on, grid on
% bode(W_c_rud)
% bode(W_c_mor)
% legend('$Wa\_a$','$Wa\_r$','$Wa\_m$','location','best','interpreter','latex')

% performance weight
W_p_bet = 1/(s/0.001+1)*r2d*20;
W_p_phi = 1/(s/0.001+1)*r2d*50;
W_p_lat = blkdiag(W_p_bet,W_p_phi);
% figure('units','centimeters','position',[39 2 13 10])
% bode(W_p_bet)
% hold on, grid on
% bode(W_p_phi)
% legend('$Wp\_\beta$','$Wp\_\phi$','location','best','interpreter','latex')

% time delay
TD70ms = pade(exp(-70e-3*s),1);
TD4 = blkdiag(TD30ms,TD30ms,TD30ms,TD70ms);

% lateral LPV modeling
systemnames         = 'P_lat W_m_lat W_n_lat W_c_lat W_p_lat W_r_lat P_a_lat TD4';
inputvar            = '[r{2} ; n{4} ; u{3}]';
outputvar           = '[W_c_lat ; W_p_lat ; W_r_lat(1)-TD4(1) ; W_r_lat(2)-TD4(4) ; TD4(2) ; TD4(3)]';
input_to_P_lat      = '[P_a_lat]';
input_to_P_a_lat    = '[u]';
input_to_W_m_lat    = '[W_r_lat]';
input_to_W_n_lat    = '[n]';
input_to_W_c_lat    = '[u]';
input_to_W_p_lat    = '[W_m_lat(1)-P_lat(1);W_m_lat(2)-P_lat(4)]';
input_to_W_r_lat    = '[r]';
input_to_TD4        = '[P_lat+W_n_lat]';
sysoutname          = 'G_lat';
cleanupsysic        = 'yes'; 
sysic
num_state_G_lat  = width(G_lat.value('rho1_sas_param',sas_grid(1),'rho2_alt_param',alt_grid(1),'rho3_mor_param',mor_grid(1)).A);
num_input_G_lat  = width(G_lat.value('rho1_sas_param',sas_grid(1),'rho2_alt_param',alt_grid(1),'rho3_mor_param',mor_grid(1)).B);
num_output_G_lat = height(G_lat.value('rho1_sas_param',sas_grid(1),'rho2_alt_param',alt_grid(1),'rho3_mor_param',mor_grid(1)).C);
fprintf('Lateral-directional generalized plant size: %ds %di %do\n',num_state_G_lat,num_input_G_lat,num_output_G_lat)

% H-infinity synthesis
ncont_lat = 3;
nmeas_lat = 4;
clearvars K_lat_hinf_grid gamma_lat_hinf_grid
tic
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor 
    G_lat_point = G_lat.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
    [~,~,gamma_lat_hinf_point_optimal]        = hinfsyn(G_lat_point,nmeas_lat,ncont_lat);
    [K_lat_hinf_point,~,gamma_lat_hinf_point] = hinfsyn(G_lat_point,nmeas_lat,ncont_lat,1.2*gamma_lat_hinf_point_optimal);
    fprintf('lat Hinf gamma = %.5g (%.3g%% back-off from %.5g)\n',gamma_lat_hinf_point,(gamma_lat_hinf_point/gamma_lat_hinf_point_optimal-1)*100,gamma_lat_hinf_point_optimal)
    K_lat_hinf_grid(:,:,i,j,k) = K_lat_hinf_point;
    gamma_lat_hinf(i,j,k) = gamma_lat_hinf_point;
end
end
end
time_lat_hinf = toc;
K_lat_hinf = pss(K_lat_hinf_grid,Domain);
save('data/lat_hinf.mat','K_lat_hinf','gamma_lat_hinf','time_lat_hinf')

% LPV synthesis
tic
[K_lat_lpv,gamma_lat_lpv,info_lat_lpv] = lpvsyn(G_lat,nmeas_lat,ncont_lat);
time_lat_lpv = toc;
save('data/lat_lpv.mat','K_lat_lpv','gamma_lat_lpv','info_lat_lpv','time_lat_lpv')
fprintf('lat LPV  gamma = %.5g (%.3g%% back-off from %.5g)\n',gamma_lat_lpv,(gamma_lat_lpv/info_lat_lpv.MinGamma-1)*100,info_lat_lpv.MinGamma)

% closed-loop interconnection with lateral-directional H_infinity controller
P_lat_scl = diag([r2d r2d r2d r2d]); % [deg] x [deg] x [deg] x [deg]
K_lat_scl = diag([1 1 1]); % [deg] x [deg] x [%]
systemnames         = 'P_lat K_lat_hinf W_r_lat P_a_lat TD4 P_lat_scl K_lat_scl';
inputvar            = '[r{2}]';
outputvar           = '[P_lat_scl ; K_lat_scl]';
input_to_P_lat      = '[P_a_lat]';
input_to_K_lat_hinf = '[W_r_lat(1)-TD4(1) ; W_r_lat(2)-TD4(4) ; TD4(2) ; TD4(3)]';
input_to_W_r_lat    = '[r]';
input_to_P_a_lat    = '[K_lat_hinf]';
input_to_TD4        = '[P_lat]';
input_to_P_lat_scl  = '[P_lat]';
input_to_K_lat_scl  = '[K_lat_hinf]';
sysoutname          = 'CL_lat_hinf';
cleanupsysic        = 'yes'; 
sysic

% closed-loop interconnection with lateral-directional LPV controller
systemnames         = 'P_lat K_lat_lpv W_r_lat P_a_lat TD4 P_lat_scl K_lat_scl';
inputvar            = '[r{2}]';
outputvar           = '[P_lat_scl ; K_lat_scl]';
input_to_P_lat      = '[P_a_lat]';
input_to_K_lat_lpv  = '[W_r_lat(1)-TD4(1) ; W_r_lat(2)-TD4(4) ; TD4(2) ; TD4(3)]';
input_to_W_r_lat    = '[r]';
input_to_P_a_lat    = '[K_lat_lpv]';
input_to_TD4        = '[P_lat]';
input_to_P_lat_scl  = '[P_lat]';
input_to_K_lat_scl  = '[K_lat_lpv]';
sysoutname          = 'CL_lat_lpv';
cleanupsysic        = 'yes'; 
sysic

% plot_lat_step_lin
% plot_lat_bode

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

%% rate-bounded design
% % longitudinal rate-bounded LPV synthesis
% b0 = basis(1,0);
% sas_param = pgrid('rho1_sas_param',sas_grid);
% alt_param = pgrid('rho2_alt_param',alt_grid);
% mor_param = pgrid('rho3_mor_param',mor_grid);
% bsas = basis(sas_param,'rho1_sas_param',1);
% balt = basis(alt_param,'rho2_alt_param',1);
% bmor = basis(mor_param,'rho3_mor_param',1);
% Xb = [b0;bsas;balt;bmor;bsas^2;balt^2;bmor^2];
% Yb = Xb;
% G_lon_RB = G_lon;
% G_lon_RB.Domain = Domain_RB;
% tic
% [K_lon_lpvr,gamma_lon_lpvr,info_lon_lpvr] = lpvsyn(G_lon_RB,nmeas_lon,ncont_lon,Xb,Yb);
% time_lon_lpvr = toc;
% save('data/lon_lpvr.mat','K_lon_lpvr','gamma_lon_lpvr','info_lon_lpvr','time_lon_lpvr')
% fprintf('Lon LPVr gamma = %.5g (%.3g%% back-off from %.5g)\n',gamma_lon_lpv,(gamma_lon_lpv/info_lon_lpv.MinGamma-1)*100,info_lon_lpv.MinGamma)
% 
% % lateral rate-bounded LPV synthesis
% b0 = basis(1,0);
% sas_param = pgrid('rho1_sas_param',sas_grid);
% alt_param = pgrid('rho2_alt_param',alt_grid);
% mor_param = pgrid('rho3_mor_param',mor_grid);
% bsas = basis(sas_param,'rho1_sas_param',1);
% balt = basis(alt_param,'rho2_alt_param',1);
% bmor = basis(mor_param,'rho3_mor_param',1);
% Xb = [b0;bsas;balt;bmor;bsas^2;balt^2;bmor^2];
% Yb = Xb;
% G_lat_RB = G_lat;
% G_lat_RB.Domain = Domain_RB;
% tic
% [K_lat_lpvr,gamma_lat_lpvr,info_lat_lpvr] = lpvsyn(G_lat_RB,nmeas_lat,ncont_lat,Xb,Yb);
% time_lat_lpvr = toc;
% save('data/lat_lpvr.mat','K_lat_lpvr','gamma_lat_lpvr','info_lat_lpvr','time_lat_lpvr')
% fprintf('lat LPVr gamma = %.5g (%.3g%% back-off from %.5g)\n',gamma_lat_lpv,(gamma_lat_lpv/info_lat_lpv.MinGamma-1)*100,info_lat_lpv.MinGamma)
% 
% % longitudinal rate-bounded LPV synthesis
% b0 = basis(1,0);
% sas_param = pgrid('rho1_sas_param',sas_grid);
% alt_param = pgrid('rho2_alt_param',alt_grid);
% mor_param = pgrid('rho3_mor_param',mor_grid);
% bsas = basis(sas_param,'rho1_sas_param',1);
% balt = basis(alt_param,'rho2_alt_param',1);
% bmor = basis(mor_param,'rho3_mor_param',1);
% Xb = [b0;bsas;balt;bmor;bsas^2;balt^2;bmor^2];
% Yb = Xb;
% G_lon_ex_RB = G_lon;
% G_lon_ex_RB.Domain = Domain_RB;
% tic
% [Klonelpvr,gamma_lon_ex_lpvr,info_lon_ex_lpvr] = lpvsyn(G_lon_ex_RB,nmeas_lon_ex,ncont_lon_ex,Xb,Yb);
% time_lon_ex_lpvr = toc;
% save('data/lon_ex_lpvr.mat','Klonelpvr','gamma_lon_ex_lpvr','info_lon_ex_lpvr','time_lon_ex_lpvr')
% fprintf('Lon Ex LPVr gamma = %.5g (%.3g%% back-off from %.5g)\n',gamma_lon_ex_lpv,(gamma_lon_ex_lpv/info_lon_ex_lpv.MinGamma-1)*100,info_lon_ex_lpv.MinGamma)
% 
% % lateral rate-bounded LPV synthesis
% b0 = basis(1,0);
% sas_param = pgrid('rho1_sas_param',sas_grid);
% alt_param = pgrid('rho2_alt_param',alt_grid);
% mor_param = pgrid('rho3_mor_param',mor_grid);
% bsas = basis(sas_param,'rho1_sas_param',1);
% balt = basis(alt_param,'rho2_alt_param',1);
% bmor = basis(mor_param,'rho3_mor_param',1);
% Xb = [b0;bsas;balt;bmor;bsas^2;balt^2;bmor^2];
% Yb = Xb;
% G_lat_ex_RB = G_lat_ex;
% G_lat_ex_RB.Domain = Domain_RB;
% tic
% [Klatelpvr,gamma_lat_ex_lpvr,info_lat_ex_lpvr] = lpvsyn(G_lat_ex_RB,nmeas_lat_ex,ncont_lat_ex,Xb,Yb);
% time_lat_ex_lpvr = toc;
% save('data/lat_ex_lpvr.mat','Klatelpvr','gamma_lat_ex_lpvr','info_lat_ex_lpvr','time_lat_ex_lpvr')
% fprintf('lat Ex LPVr gamma = %.5g (%.3g%% back-off from %.5g)\n',gamma_lat_ex_lpv,(gamma_lat_ex_lpv/info_lat_ex_lpv.MinGamma-1)*100,info_lat_ex_lpv.MinGamma)
% 
%%
save('data/lpv_design_data.mat')

% plot_parameter_grid
% plot_trim_condition
% plot_mode
% plot_lon_bode
% plot_lon_ex_bode
% plot_lon_step_lin
% plot_lon_ex_step_lin
% plot_lat_bode
% plot_lat_ex_bode
% plot_lat_step_lin
% plot_lat_ex_step_lin