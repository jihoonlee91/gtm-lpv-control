%% Setup 
% initialization
close all; clear all; clc
set(0,'DefaultLineLineWidth',1)

% simulink model setup
setup

% unit conversion parameters
knots2fps = 1.68781; % knots to feet per seconds
m2ft      = 3.28084; % meters to feet
d2r       = pi/180; % degrees to radians
r2d       = 1/d2r; % radians to degree
g         = 32.174; % gravitational acceleration [ft/s^2]

%% Domain definition
% scheduling parameter gridding
sas_grid  = 60:10:120; % [kt]
alt_grid  = 0:5000:10000; % 700~1100 [ft]
mor_grid  = -25:12.5:0; % between -25 and 0 [%]
num_sas   = numel(sas_grid); % number sas grid
num_alt   = numel(alt_grid); % number alt grid
num_mor   = numel(mor_grid); % number morphing grid

% construct Domain 
Domain = rgrid({'rho1_sas_param','rho2_alt_param','rho3_mor_param'},{sas_grid,alt_grid,mor_grid});

%% trimming
set_param('gtm_design','MinStep','auto')
set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','on') % switch off controller
set_param('gtm_design/Guidance','commented','on') % switch off controller
set_param('gtm_design/Monitor','commented','on') % switch off controller

clearvars MWS_grid 
clearvars sas_trim tas_trim alt_trim mor_trim alp_trim pitch_trim throttle_trim aileron_trim elevator_trim rudder_trim
clearvars sys_sas_trim sys_tas_trim sys_alt_trim sys_mor_trim sys_alp_trim sys_pitch_trim sys_throttle_trim sys_aileron_trim sys_elevator_trim sys_rudder_trim
for k = 1:num_mor
for j = 1:num_alt
for i = 1:num_sas
    sas = sas_grid(i); % trim sas [kt]
    alt = alt_grid(j); % trim alt [ft]
    mor = mor_grid(k); % trim mor [%]
    tas = sas2tas(sas,alt); % [kts]

    fprintf('trimming... #%d @ TAS: %d kt (SAS: %d kt), Alt: %d ft, Mor: %.3g %%\n',sub2ind([num_sas num_alt num_mor],i,j,k),tas,sas,alt,mor)
    [MWS,Xtrim,Trimcond,Err] = trimgtm_morph_try(tas0,alt0,etas0);
    MWS_grid(i,j,k) = MWS;
    
    % extract trim condition
    sas_trim(i,j,k)      = tas2sas(Trimcond.tas,Trimcond.altitude); % [kts]
    tas_trim(i,j,k)      = Trimcond.tas; % [kts]
    alt_trim(i,j,k)      = Trimcond.altitude; % [ft]
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

%% linearization
set_param('gtm_design','MinStep','auto')
set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','on') % switch off controller
set_param('gtm_design/Guidance','commented','on') % switch off controller
set_param('gtm_design/Monitor','commented','on') % switch off controller

clearvars GTM_lon GTM_lat GTM_lon_ex GTM_lat_ex
for k = 1:num_mor 
for j = 1:num_alt
for i = 1:num_sas
    sas = sas_grid(i); % trim sas [kt]
    alt = alt_grid(j); % trim alt [ft]
    mor = mor_grid(k); % trim mor [%]
    tas = sas2tas(sas,alt); % [kt]

    % linearization
    fprintf('Linearizing... #%d @ TAS: %d kt (SAS: %d kt), Alt: %d ft, Mor: %.3g %%\n',sub2ind([num_sas num_alt num_mor],i,j,k),tas,sas,alt,mor)
    [sys,londyn,latdyn]=linmodel(MWS_grid(i,j,k),1,0,0);
    
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

for k = 1:num_mor 
for j = 1:num_alt
for i = 1:num_sas
    A = GTM_lon(:,:,i,j,k).A;
    B = [GTM_lon(:,:,i,j,k).B GTM_lon(:,:,i,j,k).A(:,1) GTM_lon(:,:,i,j,k).A(:,2)/tas_trim(i,j,k)];
    C = GTM_lon(:,:,i,j,k).C;
    D = [GTM_lon(:,:,i,j,k).D GTM_lon(:,:,i,j,k).C(:,[1 2])];
    londyn_aug_w = ss(A,B,C,D);
    GTM_lon_w(:,:,i,j,k) = londyn_aug_w;
end
end
end

% construct LPV model
P_lon = pss(GTM_lon_w,Domain);
P_lat = pss(GTM_lat,Domain);
P_lon_ex = pss(GTM_lon_ex,Domain);
P_lat_ex = pss(GTM_lat_ex,Domain);

% save LPV model
save('data/lpv_model.mat','P_lon','P_lat','P_lon_ex','P_lat_ex')
% save('data/linearization_data.mat')

plot_mode

%% longitudinal autopilot
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
W_r_tas = 4*knots2fps; % tas reference scale [ft/s]
W_r_alt = 10; % altitude reference scale [ft]
W_r_lon = blkdiag(W_r_tas,W_r_alt);

% ideal response
omega_tas = 2; % ideal tas model natural frequency [rad/s]
omega_alt = 2; % ideal alt model natural frequency [rad/s]
zeta_tas = .8; % ideal tas model damping ratio []
zeta_alt = .8; % ideal alt model damping ratio []
W_m_tas = tf(omega_tas^2,[1 2*zeta_tas*omega_tas omega_tas^2]); % ideal tas model (2nd-order system)
W_m_alt = tf(omega_alt^2,[1 2*zeta_alt*omega_alt omega_alt^2]); % ideal alt model (2nd-order system)
% W_m_tas = 1;
% W_m_alt = 1;
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
W_n_alt   = 0.01*m2ft; % altitude white noise, sensor fusion (barometer + MAG3 IMU) [ft]
W_n_lon = blkdiag(W_n_tas,W_n_alp,W_n_q,W_n_theta,W_n_alt);

% control weight
W_c_eng = (s/0.5+1)/(s/120+1)*0.001; % throttle weight [%]
W_c_ele = (s/0.5+1)/(s/120+1)*0.001; % elevator weight [deg]
W_c_lon = blkdiag(W_c_eng,W_c_ele);
% figure('units','centimeters','position',[26 2 13 10])
% bodemag(W_c_eng,W_c_ele)
% grid on
% legend('$Wc\_t$','$Wc\_e$','location','best','interpreter','latex')

% performance weight
% W_p_tas = (s/omega_tas+1)/(s/0.01+1)/(W_n_tas); % tas tracking error weight
% W_p_alt = (s/omega_alt+1)/(s/0.01+1)/(W_n_alt); % alt tracking error weight
W_p_tas = (s/120+1)/(s/0.005+1)/(0.06*knots2fps*2); % tas tracking error weight
W_p_alt = (s/120+1)/(s/0.005+1)/(0.01*m2ft*2); % alt tracking error weight
W_p_lon = blkdiag(W_p_tas,W_p_alt);
% figure('units','centimeters','position',[39 2 13 10])
% bodemag(W_p_tas,W_p_alt)
% grid on
% legend('$Wp\_V$','$Wp\_h$','location','best','interpreter','latex')

% time delay
TD30ms = pade(exp(-30e-3*s),1);
TD70ms = pade(exp(-70e-3*s),1);
TD5 = blkdiag(TD30ms,TD30ms,TD30ms,TD70ms,TD70ms);

% TD_UL = eye(2)*pade(exp(-5e-3*s),1);
% TD_Act = eye(2)*pade(exp(-10e-3*s),1);
% TD_DL = blkdiag(pade(exp(-15e-3*s),1),pade(exp(-15e-3*s),1),pade(exp(-15e-3*s),1),pade(exp(-45e-3*s),1),pade(exp(-45e-3*s),1));
% TD_UL = eye(2);
% TD_Act = eye(2);
% TD_DL = eye(5);

D_eng = 1/(s/0.5+1)*0.01;
D_ele = 1/(s/0.5+1)*0.01;
D_lon = blkdiag(D_eng,D_ele);

D_w_u = 1/(s/0.5+1)*0.2*1e-5;
D_w_w = 1/(s/0.5+1)*0.017*1e-5;
D_w_lon = blkdiag(D_w_u,D_w_w);

% systemnames         = 'P_lon W_m_lon W_n_lon W_c_lon W_p_lon W_r_lon P_a_lon TD5';
% inputvar            = '[r{2} ; n{5} ; u{2}]';
% outputvar           = '[W_c_lon ; W_p_lon ; W_r_lon(1)-TD5(1) ; W_r_lon(2)-TD5(5) ; TD5(2) ; TD5(3) ; TD5(4)]';
% input_to_P_lon      = '[P_a_lon]';
% input_to_W_m_lon    = '[W_r_lon]';
% input_to_W_n_lon    = '[n]';
% input_to_W_c_lon    = '[u]';
% input_to_W_p_lon    = '[W_m_lon(1)-P_lon(1) ; W_m_lon(2)-P_lon(5)]';
% input_to_W_r_lon    = '[r]';
% input_to_P_a_lon    = '[u]';
% input_to_TD5        = '[P_lon+W_n_lon]';
% sysoutname          = 'G_lon';
% cleanupsysic        = 'yes';
% sysic

systemnames         = 'P_lon W_m_lon W_n_lon W_c_lon W_p_lon W_r_lon P_a_lon TD5 D_lon';
inputvar            = '[r{2} ; n{5} ; d{2} ; u{2}]';
outputvar           = '[W_c_lon ; W_p_lon ; W_r_lon(1)-TD5(1) ; W_r_lon(2)-TD5(5) ; TD5(2) ; TD5(3) ; TD5(4)]';
input_to_P_lon      = '[P_a_lon+D_lon]';
input_to_W_m_lon    = '[W_r_lon]';
input_to_W_n_lon    = '[n]';
input_to_W_c_lon    = '[u]';
input_to_W_p_lon    = '[W_m_lon(1)-P_lon(1) ; W_m_lon(2)-P_lon(5)]';
input_to_W_r_lon    = '[r]';
input_to_P_a_lon    = '[u]';
input_to_TD5        = '[P_lon+W_n_lon]';
input_to_D_lon      = '[d]';
sysoutname          = 'G_lon';
cleanupsysic        = 'yes';
sysic

% systemnames         = 'P_lon W_m_lon W_n_lon W_c_lon W_p_lon W_r_lon P_a_lon TD5 D_lon D_w_lon';
% inputvar            = '[r{2} ; n{5} ; d{2}; d_w{2} ; u{2}]';
% outputvar           = '[W_c_lon ; W_p_lon ; W_r_lon(1)-TD5(1) ; W_r_lon(2)-TD5(5) ; TD5(2) ; TD5(3) ; TD5(4)]';
% input_to_P_lon      = '[P_a_lon+D_lon ; D_w_lon]';
% input_to_W_m_lon    = '[W_r_lon]';
% input_to_W_n_lon    = '[n]';
% input_to_W_c_lon    = '[u]';
% input_to_W_p_lon    = '[W_m_lon(1)-P_lon(1) ; W_m_lon(2)-P_lon(5)]';
% input_to_W_r_lon    = '[r]';
% input_to_P_a_lon    = '[u]';
% input_to_TD5        = '[P_lon+W_n_lon]';
% input_to_D_lon      = '[d]';
% input_to_D_w_lon    = '[d_w]';
% sysoutname          = 'G_lon';
% cleanupsysic        = 'yes';
% sysic

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
    fprintf('lon Hinf #%d gamma = %.5g (%.3g%% back-off from %.5g)\n',sub2ind([num_mor num_alt num_sas],k,j,i),gamma_lon_hinf_point,(gamma_lon_hinf_point/gamma_lon_hinf_point_optimal-1)*100,gamma_lon_hinf_point_optimal)
    K_lon_hinf_grid(:,:,i,j,k) = K_lon_hinf_point;
    gamma_lon_hinf(i,j,k) = gamma_lon_hinf_point;
end
end
end
time_lon_hinf = toc;
fprintf('Lon Hinf Synthesis Time: %.1f seconds \n',time_lon_hinf)
K_lon_hinf = pss(K_lon_hinf_grid,Domain);

% LPV synthesis
tic
[K_lon_lpv,gamma_lon_lpv,info_lon_lpv] = lpvsyn(G_lon,nmeas_lon,ncont_lon);
time_lon_lpv = toc;
fprintf('Lon LPV  Synthesis Time: %.1f seconds \n',time_lon_lpv)
fprintf('Lon LPV  gamma = %.5g (%.3g%% back-off from %.5g)\n',gamma_lon_lpv,(gamma_lon_lpv/info_lon_lpv.MinGamma-1)*100,info_lon_lpv.MinGamma)

% closed-loop interconnection with Longitudinal H_infinity controller
P_lon_scl = diag([1/knots2fps r2d r2d r2d 1]); % [kt] x [deg] x [deg] x [deg] x [ft]
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

save('data/lon_hinf.mat','K_lon_hinf','gamma_lon_hinf','time_lon_hinf')
save('data/lon_lpv.mat','K_lon_lpv','gamma_lon_lpv','info_lon_lpv','time_lon_lpv')


%% lateral-directional autopilot
% actuator model
P_a_ail = 2*pi*5/(s+2*pi*5); % Bandwidth 5 Hz and 5+10 ms time delay
P_a_rud = 2*pi*5/(s+2*pi*5); % Bandwidth 5 Hz and 5+10 ms time delay
P_a_mor = 2*pi*0.5/(s+2*pi*0.5); % Bandwidth 5 Hz and 5+10 ms time delay
P_a_lat = blkdiag(P_a_ail,P_a_rud,P_a_mor);

% maximum expected command
W_r_bet = 3*d2r;
W_r_phi = 30*d2r;
W_r_lat = blkdiag(W_r_bet,W_r_phi);

% ideal model
omega_bet = 2; % [rad/s]
zeta_bet = .8; % []
W_m_bet = tf(omega_bet^2,[1 2*zeta_bet*omega_bet omega_bet^2]);
omega_phi = 5; % [rad/s]
zeta_phi = .8; % []
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
W_c_ail = (s/.1+1)/(s/120+1)*0.0001; % [deg]
W_c_rud = (s/.1+1)/(s/120+1)*0.0005; % [deg]
W_c_mor = (s/.1+1)/(s/120+1)*0.01; % [deg]
W_c_lat = blkdiag(W_c_ail,W_c_rud,W_c_mor);
% figure('units','centimeters','position',[26 2 13 10])
% bode(W_c_ail)
% hold on, grid on
% bode(W_c_rud)
% bode(W_c_mor)
% legend('$Wa\_a$','$Wa\_r$','$Wa\_m$','location','best','interpreter','latex')

% performance weight
W_p_bet = (s/120+1)/(s/0.1+1)/(W_n_bet*2);
W_p_phi = (s/120+1)/(s/0.003+1)/(W_n_phi*2);
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
    fprintf('lat Hinf #%d gamma = %.5g (%.3g%% back-off from %.5g)\n',sub2ind([num_mor num_alt num_sas],k,j,i),gamma_lat_hinf_point,(gamma_lat_hinf_point/gamma_lat_hinf_point_optimal-1)*100,gamma_lat_hinf_point_optimal)
    K_lat_hinf_grid(:,:,i,j,k) = K_lat_hinf_point;
    gamma_lat_hinf(i,j,k) = gamma_lat_hinf_point;
end
end
end
time_lat_hinf = toc;
fprintf('Lat Hinf Synthesis Time: %.1f seconds \n',time_lat_hinf)
K_lat_hinf = pss(K_lat_hinf_grid,Domain);

% LPV synthesis
tic
[K_lat_lpv,gamma_lat_lpv,info_lat_lpv] = lpvsyn(G_lat,nmeas_lat,ncont_lat);
time_lat_lpv = toc;
fprintf('Lat LPV  Synthesis Time: %.1f seconds \n',time_lat_lpv)
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

%% longitudinal CAS
s = tf('s'); % Laplace variable

% actuator model
P_a_ele = 2*pi*5/(s+2*pi*5); % elevator model
P_a_lon_ex = blkdiag(P_a_ele); % actuator model

% reference scale (maximum expected command)
W_r_an  = 3*g; % an reference scale [ft/s^2]
W_r_lon_ex = blkdiag(W_r_an);

% ideal response
omega_an = 2.5;
zeta_an = .8;
W_m_an  = omega_an^2/(s^2+2*zeta_an*omega_an*s+omega_an^2); % ideal an model (1st-order system)
W_m_lon_ex = blkdiag(W_m_an); % ideal response

% sensor measurement noise
W_n_alp = 0.031*d2r; % 0.031 deg alp white noise (ADS)
W_n_q   = 0.3857*d2r; % 0.3857 deg/s q white noise (MAG3)
W_n_an  = 0.0082*g; % 0.0082g white noise (MAG3)
W_n_lon_ex = blkdiag(W_n_alp,W_n_q,W_n_an);

% control weight
W_c_ele = (s/0.5+1)/(s/120+1)*0.0001; % elevator weight [deg]
W_c_lon_ex = blkdiag(W_c_ele);

% performance weight
W_p_an = (s/omega_an+1)/(s/0.04+1)/(0.0082*g*5); % an tracking error weight
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
    fprintf('lon Ex Hinf #%d gamma = %.5g (%.3g%% back-off from %.5g)\n',sub2ind([num_mor num_alt num_sas],k,j,i),gamma_lon_ex_hinf_point,(gamma_lon_ex_hinf_point/gamma_lon_ex_hinf_point_optimal-1)*100,gamma_lon_ex_hinf_point_optimal)
    Klonexhinf_grid(:,:,i,j,k) = Klonexhinf_point;
    gamma_lon_ex_hinf(i,j,k) = gamma_lon_ex_hinf_point;
end
end
end
time_lon_ex_hinf = toc;
fprintf('Lon Ex Hinf Synthesis Time: %.1f seconds \n',time_lon_ex_hinf)
Klonexhinf = pss(Klonexhinf_grid,Domain);

% LPV synthesis
tic
[Klonexlpv,gamma_lon_ex_lpv,info_lon_ex_lpv] = lpvsyn(G_lon_ex,nmeas_lon_ex,ncont_lon_ex);
time_lon_ex_lpv = toc;
fprintf('Lon Ex LPV  Synthesis Time: %.1f seconds \n',time_lon_ex_lpv)
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

%% lateral-directional CAS
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
zeta_bet = .8; % []
W_m_bet = tf(omega_bet^2,[1 2*zeta_bet*omega_bet omega_bet^2]);
omega_p = 10; % [rad/s]
zeta_p = .8; % []
W_m_p = tf(omega_p^2,[1 2*zeta_p*omega_p omega_p^2]);
W_m_lat_ex = blkdiag(W_m_bet,W_m_p);
% figure
% step(W_m_p)

% noise model
W_n_bet = 0.033*d2r; % 0.033 deg
W_n_p   = 0.5220*d2r; % 0.5220 deg/sec
W_n_r   = 0.3330*d2r; % 0.3330 deg/sec
W_n_lat_ex = blkdiag(W_n_bet,W_n_p,W_n_r);

% control weight
% W_c_ail = (s/31.4+1)/(s/120+1)*0.008; % [deg]
% W_c_rud = (s/.1+1)/(s/120+1)*0.0001; % [deg]
% W_c_mor = (s/3.14+1)/(s/120+1)*0.01; % [deg]

W_c_ail = (s/.1+1)/(s/120+1)*0.0001; % [deg]
W_c_rud = (s/.1+1)/(s/120+1)*0.0005; % [deg]
W_c_mor = (s/.1+1)/(s/120+1)*0.01; % [deg]

W_c_lat_ex = blkdiag(W_c_ail,W_c_rud,W_c_mor);

% performance weight
W_p_bet = (s/120+1)/(s/0.1+1)/(W_n_bet*2);
W_p_p = (s/120+1)/(s/0.1+1)/(W_n_p*2);
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
    fprintf('lat Ex Hinf #%d gamma = %.5g (%.3g%% back-off from %.5g)\n',sub2ind([num_mor num_alt num_sas],k,j,i),gamma_lat_ex_hinf_point,(gamma_lat_ex_hinf_point/gamma_lat_ex_hinf_point_optimal-1)*100,gamma_lat_ex_hinf_point_optimal)
    Klatexhinf_grid(:,:,i,j,k) = Klatexhinf_point;
    gamma_lat_ex_hinf(i,j,k) = gamma_lat_ex_hinf_point;
end
end
end
time_lat_ex_hinf = toc;
fprintf('Lat Ex Hinf Synthesis Time: %.1f seconds \n',time_lat_ex_hinf)
Klatexhinf = pss(Klatexhinf_grid,Domain);

% LPV synthesis
tic
[Klatexlpv,gamma_lat_ex_lpv,info_lat_ex_lpv] = lpvsyn(G_lat_ex,nmeas_lat_ex,ncont_lat_ex);
time_lat_ex_lpv = toc;
fprintf('Lat Ex LPV  Synthesis Time: %.1f seconds \n',time_lat_ex_lpv)
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

%%
save('data/lpv_design_data.mat')