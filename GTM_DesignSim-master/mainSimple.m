%% Initialization
close all
clear all
clc

% Unit conversion parameters
knots2fps = 1.68781; % knots to feet per seconds
m2ft      = 3.28084; % meters to feet
d2r       =pi/180; % degrees to radians

% Model basic setup
setup
MWS = grabmws('gtm_design');

%% Basic setup
% Parameter grid
sas_grid  = [85]; % [kts]
alt_grid  = [0]; % 700~1100 [ft]
mor_grid  = [-12.5]; % between -25 and 0 [%]
num_sas   = numel(sas_grid); % number sas grid
num_alt   = numel(alt_grid); % number alt grid
num_mor   = numel(mor_grid); % number morphing grid

% Parameters for rate-bounded design
% g = 32.174; % gravitational acceleration [ft/s^2]
% Coeff_throttle = [0.000033035538475  -0.008354385314499 1.191126728328792 31.518959847101669];
% Coeff_thrust   = [0.000014847593013,  0, 0, 0.412662356277121];
% T_max = polyval(Coeff_thrust,polyval(Coeff_throttle,100)); % Maximum thrust in [lbf] at sea level without ram drag
% sas_RateBound = 10; % 2*T_max/MWS.w0 = 0.53g = 17 ft/s^2
% alt_RateBound = 20; % convvel([100],'kts','ft/s')
% mor_RateBound = 0.025; % 10 sec for full deployment

Domain = rgrid({'rho1_sas_param','rho2_alt_param','rho3_mor_param'},{sas_grid,alt_grid,mor_grid});
% DomainRB = rgrid({'rho1_sas_param','rho2_alt_param','rho3_mor_param'},{sas_grid,alt_grid,mor_grid},[[-1,1]*sas_RateBound;[-1,1]*alt_RateBound;[-1,1]*mor_RateBound]);

%% Trim calculation
set_param('gtm_design','MinStep','auto')
set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','on') % switch off controller

PitchSurf = 'elev'; % pitch surface
verbose   = 0; % soln disp

if isfile('./result/Trim.mat') == 0
    clearvars MWS_grid sas_trim tas_trim alt_trim mor_trim alp_trim pitch_trim throttle_trim aileron_trim elevator_trim rudder_trim
    clearvars sys_alp_trim sys_pitch_trim sys_throttle_trim sys_aileron_trim sys_elevator_trim sys_rudder_trim
    count = 0;
    for i = 1:num_sas
    for j = 1:num_alt
    for k = 1:num_mor
        sas = sas_grid(i); % current sas [kts]
        alt = alt_grid(j); % current alt [ft]
        mor = mor_grid(k); % current mor []
        tas = sas2tas(sas,alt); % [kts]

        target            = struct('tas',tas,'gamma',0); % target trim condition - straight and level flight, heading east [knot] X [deg]
        MWS.StatesInp(9)  = alt; % set trim altitude [ft]
        MWS.bias.morphing = mor; % set trim symmetric morphing parameter [%]

        loadmws(MWS,'gtm_design'); % load MWS

        [MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose); % trim calculation (attempt #1)
        if (Err > 1e-3)
            loadmws(init_design(),'gtm_design');
            warning('   Poor convergence, trying again.\n');
            [MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose); % trim calculation (attempt #2)
            if (Err > 1e-3)
                loadmws(init_design(),'gtm_design');
                warning('   Poor convergence, trying again.\n');
                [MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose); % trim calculation (attempt #3)
                if (Err > 1e-3)
                    error('TRIM FAIL')
                end
            end
        end
        count = count+1;
        fprintf('TRIM #%d @ TAS: %d, Alt: %d, Mor: %.3g\n',count,tas,alt,mor)

        % save trim results
        MWS_grid(i,j,k)      = MWS;

        sas_trim(i,j,k)      = sas; % [kts]
        tas_trim(i,j,k)      = tas; % [kts]
        alt_trim(i,j,k)      = alt; % [ft]
        mor_trim(i,j,k)      = mor; % []

        alp_trim(i,j,k)      = Trimcond.alpha; % [deg]
        pitch_trim(i,j,k)    = Trimcond.pitch; % [deg]
        throttle_trim(i,j,k) = Trimcond.throttle; % [%]
        aileron_trim(i,j,k)  = Trimcond.aileron; % [deg]
        elevator_trim(i,j,k) = Trimcond.elevator; % [deg]
        rudder_trim(i,j,k)   = Trimcond.rudder; % [deg]

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

    save Trim.mat MWS_grid sch_sas_trim sch_tas_trim sch_alt_trim sch_mor_trim sch_alp_trim sch_pitch_trim sch_throttle_trim sch_aileron_trim sch_elevator_trim sch_rudder_trim tas_trim alt_trim mor_trim
    disp('Trim Data Saved')
else
    load Trim.mat
    disp('Trim Data Loaded')
end

%% Linearization
set_param('gtm_design','MinStep','auto')
set_param('gtm_design','ReturnWorkspaceOutputs','off')
set_param('gtm_design/Control/Controller','commented','on') % switch off controller

if isfile('./result/Linearization.mat') == 0
    clearvars GTM_lon GTM_lat
    count = 0;
    for i = 1:num_sas
    for j = 1:num_alt
    for k = 1:num_mor 
        sas = sas_grid(i); % current sas [kts]
        alt = alt_grid(j); % current alt [ft]
        mor = mor_grid(k); % current mor []
        tas = sas2tas(sas,alt); % [kts]

        % linearization
        % vabflag        = 1; % wind-axis linear state (TAS, alpha, beta instead of ub, vb, wb)
        % use_all_inputs = 0; % control surface as groups
        % Ts             = 0; % continuous-time model
        [sys,londyn,latdyn]=linmodel(MWS_grid(i,j,k),1,0,0);
        count = count+1;
        fprintf('LINEARIZATION #%d @ TAS: %d, Alt: %d, Mor: %.3g\n',count,tas,alt,mor)

        % Redefine input name
        set(londyn,'Inputname',{'Elevator','L Morphing','R Morphing','L Throttle','R Throttle'});
        set(latdyn,'Inputname',{'Aileron','Rudder','L Morphing','R Morphing'});
        
        % Units
        % Lon state: tas [ft/s], alpha [rad], q [rad/s], theta [rad]
        % Lon input: Elevator [deg], L Morphing [%], R Morphing [%], L Throttle [%], R Throttle [%]
        % Lat state: beta [rad], p [rad/s], r [rad/s], phi [rad]
        % Lat input: Aileron [deg], Rudder [deg], L Morphing [deg], R Morphing [deg]
    
        % Dynamics augmentation
        Alon = [londyn.A zeros(4,1);
                0 -tas 0 tas 0]; % altitude [ft]
        Blon = [londyn.B(:,4)+londyn.B(:,5) londyn.B(:,1);
                0 0];
        Clon = eye(5);
        Dlon = 0;
        Alat = latdyn.A;
        Blat = [latdyn.B(:,1:2) (latdyn.B(:,4)-latdyn.B(:,3))/2]; % (eta_R-eta_L)/2
        Clat = eye(4);
        Dlat = 0;
        
        % Form state-space model
        londyn_aug = ss(Alon,Blon,Clon,Dlon);
        set(londyn_aug,'Statename',{'tas','alpha','q','theta','alt'},...
                       'Inputname',{'delt','dele'},...
                       'Outputname',{'tas','alpha','q','theta','alt'})
        latdyn_aug = ss(Alat,Blat,Clat,Dlat);
        set(latdyn_aug,'Statename',{'beta','p','r','phi'},...
                       'Inputname',{'dela','delr','delm'},...
                       'Outputname',{'beta','p','r','phi'})
        
        % stability check
        if ~isstable(londyn_aug) % unstable
            warning('Unstable Longitudinal dynamics') 
        end
        if ~isstable(latdyn_aug) % unstable
            warning('Unstable lateral dynamics')
        end
        
        % controllability check
        if rank(ctrb(londyn_aug)) < height(londyn_aug) % uncontrollable
            error('Uncontrollable longitudinal dynamics')
        end
        if rank(ctrb(latdyn_aug)) < height(latdyn_aug) % uncontrollable
            error('Uncontrollable lateral dynamics')
        end

        % LTI model stacking
        GTM_lon(:,:,i,j,k) = londyn_aug;
        GTM_lat(:,:,i,j,k) = latdyn_aug;
    end
    end
    end
    
    % LPV model
    P_lon = pss(GTM_lon,Domain);
    P_lat = pss(GTM_lat,Domain);

    save Linearization.mat P_lon P_lat GTM_lon GTM_lat
    disp('Linearization Data Saved')
else
    load Linearization.mat
    disp('Linearization Data Loaded')
end

%% Component modeling (longitudinal performance)
s = tf('s');

% W_a (actuator dynamics)
P_a_eng = tf(ss([-1.336,-0.7314;1,0;],[1;0],[-0.1474 0.7314],0)); % Throttle Lag Model
P_a_ele = 2*pi*5/(s+2*pi*5); % Bandwidth 5 Hz and 5+10 ms time delay
P_a_lon = blkdiag(P_a_eng,P_a_ele);
% figure
% step(P_a_eng)
% hold on, grid on
% step(P_a_ele)
% legend('Pa_{eng}','Pa_{ele}')

% W_r (maximum expected command)
W_r_tas = 10*knots2fps; % 10 knot command
W_r_alt = 10; % 10 ft command
W_r_lon = blkdiag(W_r_tas,W_r_alt);

% W_m (ideal model)
omega_tas = .1; % [rad/s]
zeta_tas = 1; % []
W_m_tas = tf(omega_tas^2,[1 2*zeta_tas*omega_tas omega_tas^2]);
omega_alt = .35; % [rad/s]
zeta_alt = 1; % []
W_m_alt = tf(omega_alt^2,[1 2*zeta_alt*omega_alt omega_alt^2]);
W_m_lon = blkdiag(W_m_tas,W_m_alt);

% sensor noise (white)
W_n_tas     = 0.06*knots2fps; % 0.06 knots broadband (ADS)
W_n_alp     = 0.031*d2r; % 0.031 deg broadband (ADS)
W_n_q       = 0.3857*d2r; % 0.3857 deg/s broadband (MAG3)
W_n_theta   = 0.02*d2r; % 0.02 deg broadband (MIDG II)
W_n_alt     = 0.1; % [ft] (MIDG II GPS)
W_n_lon = blkdiag(W_n_tas,W_n_alp,W_n_q,W_n_theta,W_n_alt);

% control weight
W_c_eng = (s/0.5+1)/(s/100+1)*0.1; % [deg]
W_c_ele = (0.8*s+42.91)/(s+119.2)*0.1; % [deg]
W_c_lon = blkdiag(W_c_eng,W_c_ele);
% figure
% bode(W_c_eng)
% hold on, grid on
% bode(W_c_ele)
% legend('Wc_{eng}','Wc_{ele}')

% performance weight
W_p_tas = (s/100+1)/(s/0.005+1)/(knots2fps*10)*100;
W_p_alt = (s/100+1)/(s/0.005+1)/(knots2fps*10)*1000;
W_p_lon = blkdiag(W_p_tas,W_p_alt);
% figure
% bode(W_p_tas)
% hold on, grid on
% bode(W_p_gam)
% legend('Wp_{tas}','Wp_{gam}')

% control interconnection
systemnames         = 'P_lon W_m_lon W_n_lon W_c_lon W_p_lon W_r_lon P_a_lon';
inputvar            = '[r{2} ; n{5} ; u{2}]';
outputvar           = '[W_c_lon ; W_p_lon ; W_r_lon(1)-P_lon(1)-W_n_lon(1) ; W_r_lon(2)-P_lon(5)-W_n_lon(5) ; P_lon(2)+W_n_lon(2) ; P_lon(3)+W_n_lon(3) ; P_lon(4)+W_n_lon(4)]';
input_to_P_lon      = '[P_a_lon]';
input_to_W_m_lon    = '[W_r_lon]';
input_to_W_n_lon    = '[n]';
input_to_W_c_lon    = '[u]';
input_to_W_p_lon    = '[W_m_lon(1)-P_lon(1) ; W_m_lon(2)-P_lon(5)]';
input_to_W_r_lon    = '[r]';
input_to_P_a_lon    = '[u]';
sysoutname          = 'G_lon';
cleanupsysic        = 'yes';
sysic
% size(G_lon.value('rho1_sas_param',sas_grid(1),'rho2_alt_param',alt_grid(1),'rho3_mor_param',mor_grid(1)))

% Longitudinal H-infinity synthesis
ncont_lon = 2;
nmeas_lon = 5;
if isfile('./result/K_lon_hinf.mat') == 0
    clearvars K_lon_hinfs gamma_lon_hinfs
    for i = 1:num_sas
    for j = 1:num_alt
    for k = 1:num_mor 
        G_lon_LTI = G_lon.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
        tic
        [K_lon_hinf_point,CL,gamma_lon_hinf_point] = hinfsyn(G_lon_LTI,nmeas_lon,ncont_lon);
        SynTime_lon_hinfs(i,j,k) = toc;
        disp(['Gamma (Longitudinal H_infinity) = ',num2str(gamma_lon_hinf_point)])
        gamma_lon_hinfs(i,j,k) = gamma_lon_hinf_point;
        K_lon_hinfs(:,:,i,j,k) = K_lon_hinf_point;
    end
    end
    end
    K_lon_hinf = pss(K_lon_hinfs,Domain);
    save K_lon_hinf.mat K_lon_hinf gamma_lon_hinfs SynTime_lon_hinfs
else
    disp('Longitudinal H_infinity Synthesis Skipped & Data Loaded')
    load('./result/K_lon_hinf.mat')
end

% Closed-loop interconnection with Longitudinal H_infinity controller
systemnames         = 'P_lon K_lon_hinf W_r_lon P_a_lon';
inputvar            = '[r{2}]';
outputvar           = '[P_lon ; K_lon_hinf]';
input_to_P_lon      = '[P_a_lon]';
input_to_K_lon_hinf = '[W_r_lon(1)-P_lon(1) ; W_r_lon(2)-P_lon(5) ; P_lon(2) ; P_lon(3) ; P_lon(4)]';
input_to_W_r_lon    = '[r]';
input_to_P_a_lon    = '[K_lon_hinf]';
sysoutname          = 'CL_lon_hinf';
cleanupsysic        = 'yes'; 
sysic

% Longitudinal LPV synthesis
if isfile('./result/K_lon_lpv.mat') == 0
    opt_lpv = lpvsynOptions('BackOffFactor',1.2);
    tic
    [K_lon_lpv,gamma_lon_lpv,info_lon_lpv] = lpvsyn(G_lon,nmeas_lon,ncont_lon,opt_lpv);
    SynTime_lon_lpv = toc;
    save K_lon_lpv.mat K_lon_lpv gamma_lon_lpv info_lon_lpv SynTime_lon_lpv
else
    load('./result/K_lon_lpv.mat')
end
disp(['Min Gamma (Longitudinal Rate-Unbounded LPV) = ',num2str(info_lon_lpv.MinGamma)])
disp(['Gamma (Longitudinal Rate-Unbounded LPV) = ',num2str(gamma_lon_lpv)])
disp(['Back-Off Factor (Longitudinal Rate-Unbounded LPV) = ',num2str((gamma_lon_lpv/info_lon_lpv.MinGamma-1)*100),' %'])

% Closed-loop interconnection with Longitudinal LPV controller
systemnames         = 'P_lon K_lon_lpv W_r_lon P_a_lon';
inputvar            = '[r{2}]';
outputvar           = '[P_lon ; K_lon_lpv]';
input_to_P_lon      = '[P_a_lon]';
input_to_K_lon_lpv  = '[W_r_lon(1)-P_lon(1) ; W_r_lon(2)-P_lon(5) ; P_lon(2) ; P_lon(3) ; P_lon(4)]';
input_to_W_r_lon    = '[r]';
input_to_P_a_lon    = '[K_lon_lpv]';
sysoutname          = 'CL_lon_lpv';
cleanupsysic        = 'yes'; 
sysic

figure
step(CL_lon_hinf);
hold on, grid on
step(CL_lon_lpv);
legend('H\infty','LPV')
drawnow

%% Longitudinal rate-bounded LPV synthesis
% Basis function
% b0 = basis(1,0);
% tas_param = pgrid('rho1_tas_param',tas_grid);
% alt_param = pgrid('rho2_alt_param',alt_grid);
% mor_param = pgrid('rho3_mor_param',mor_grid);
% btas = basis(tas_param,'rho1_tas_param',1);
% balt = basis(alt_param,'rho2_alt_param',1);
% bmor = basis(mor_param,'rho3_mor_param',1);
% Xb = [b0;btas;balt;bmor;btas^2;balt^2;bmor^2];
% Yb = Xb;
% 
% G_lon.Domain = DomainRB;
% if isfile('./result/K_lon_rblpv.mat') == 0
%     disp('Longitudinal RBLPV Synthess Finished')
%     datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss Z')
%     tic
%     [K_lon_rblpv,gamma_lon_rblpv,info_lon_rblpv] = lpvsyn(G_lon,6,2,Xb,Yb,opt_lpv); % Elapsed time is 366 seconds with 4 basis functions
%     SynTime_lon_rblpv = toc;
%     disp('Longitudinal RBLPV Synthess Finished')
%     disp(['Synthesis Time: ',num2str(SynTime_lon_lpv),' sec'])
%     datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss Z')
%     save K_lon_rblpv.mat K_lon_rblpv gamma_lon_rblpv info_lon_rblpv SynTime_lon_rblpv
% else
%     disp('Longitudinal Rate-Bounded LPV Synthesis Skipped & Data Loaded')
%     load('./result/K_lon_rblpv.mat')
% end
% G_lon.Domain = Domain;
% disp(['Min Gamma (Longitudinal Rate-Bounded LPV) = ',num2str(info_lon_rblpv.MinGamma)])
% disp(['Gamma (Longitudinal Rate-Bounded LPV) = ',num2str(gamma_lon_rblpv)])
% disp(['Back-Off Factor (Longitudinal Rate-Bounded LPV) = ',num2str((gamma_lon_rblpv/info_lon_rblpv.MinGamma-1)*100),' %'])

% P_lon.Domain = DomainRB;
% systemnames         = 'P_lon P_a_lon H_s_lon K_lon_rblpv';
% inputvar            = '[r{2}]';
% outputvar           = '[P_lon]';
% input_to_P_lon      = '[P_a_lon]';
% input_to_P_a_lon    = '[K_lon_rblpv]';
% input_to_H_s_lon    = '[P_lon]';
% input_to_K_lon_rblpv  = '[r;H_s_lon]';
% sysoutname          = 'CL_lon_lpv';
% cleanupsysic        = 'yes'; 
% sysic
% 
% figure
% step(CL_lon_lpv,10)
% grid on

%% Component modeling (lateral performance)
% Actuator model
P_a_ail = 2*pi*5/(s+2*pi*5); % Bandwidth 5 Hz and 5+10 ms time delay
P_a_rud = 2*pi*5/(s+2*pi*5); % Bandwidth 5 Hz and 5+10 ms time delay
P_a_mor = 2*pi*0.5/(s+2*pi*0.5); % Bandwidth 5 Hz and 5+10 ms time delay
P_a_lat = blkdiag(P_a_ail,P_a_rud,P_a_mor);

% Maximum expected command
W_r_bet = 2*d2r;
W_r_phi = 20*d2r;
W_r_lat = blkdiag(W_r_bet,W_r_phi);

% Ideal model
omega_bet = 1.5; % [rad/s]
zeta_bet = .7; % []
W_m_bet = tf(omega_bet^2,[1 2*zeta_bet*omega_bet omega_bet^2]);
omega_phi = 3.5; % [rad/s]
zeta_phi = .7; % []
W_m_phi = tf(omega_phi^2,[1 2*zeta_phi*omega_phi omega_phi^2]);
W_m_lat = blkdiag(W_m_bet,W_m_phi);
% figure
% step(W_m_bet)
% hold on, grid on
% step(W_m_phi)
% legend('Wm_{bet}','Wm_{phi}')

W_n_bet = 0.033*d2r; % 0.033 deg
W_n_p   = 0.5220*d2r; % 0.5220 deg/sec
W_n_r   = 0.3330*d2r; % 0.3330 deg/sec
W_n_phi = 0.02*d2r; % 0.02 deg
W_n_lat = blkdiag(W_n_bet,W_n_p,W_n_r,W_n_phi);

W_c_ail = (0.8*s+42.91)/(s+119.2)*0.1; % [deg]
W_c_rud = (0.8*s+42.91)/(s+119.2)*0.1; % [deg]
W_c_mor = (0.8*s+4.291)/(s+11.92)*0.1; % [deg]
W_c_lat = blkdiag(W_c_ail,W_c_rud,W_c_mor);
figure
bode(W_c_ail)
hold on, grid on
bode(W_c_rud)
bode(W_c_mor)
legend('Wc_{ail}','Wc_{rud}','Wc_{mor}')

W_p_bet = 200*(s/100+1)/(s/0.005+1);
W_p_phi = 200*(s/100+1)/(s/0.005+1);
W_p_lat = blkdiag(W_p_bet,W_p_phi);
% figure
% bode(W_p_bet)
% hold on, grid on
% bode(W_p_phi)
% legend('Wp_{bet}','Wp_{phi}')

% Lateral LPV modeling
systemnames         = 'P_lat W_m_lat W_n_lat W_c_lat W_p_lat W_r_lat P_a_lat';
inputvar            = '[r{2} ; n{4} ; u{3}]';
outputvar           = '[W_c_lat ; W_p_lat ; W_r_lat(1)-P_lat(1)-W_n_lat(1) ; W_r_lat(2)-P_lat(4)-W_n_lat(4) ; P_lat(2)+W_n_lat(2) ; P_lat(3)+W_n_lat(3)]';
input_to_P_lat      = '[P_a_lat]';
input_to_P_a_lat    = '[u]';
input_to_W_m_lat    = '[W_r_lat]';
input_to_W_n_lat    = '[n]';
input_to_W_c_lat    = '[u]';
input_to_W_p_lat    = '[W_m_lat(1)-P_lat(1);W_m_lat(2)-P_lat(4)]';
input_to_W_r_lat    = '[r]';
sysoutname          = 'G_lat';
cleanupsysic        = 'yes'; 
sysic
% size(G_lat.value('rho1_sas_param',sas_grid(1),'rho2_alt_param',alt_grid(1),'rho3_mor_param',mor_grid(1)))

% Lateral H_infinity synthesis
nmeas_lat = 4;
ncont_lat = 3;
if isfile('./result/K_lat_hinf.mat') == 0
    clearvars K_lat_hinfs gamma_lat_hinfs
    for i = 1:num_sas
    for j = 1:num_alt
    for k = 1:num_mor 
        G_lat_LTI = G_lat.value('rho1_sas_param',sas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
        tic
        [K_lat_hinf_point,~,gamma_lat_hinf_point] = hinfsyn(G_lat_LTI,nmeas_lat,ncont_lat);
        SynTime_lat_hinfs(i,j,k) = toc;
        disp(['Gamma (Lateral H_infinity) = ',num2str(gamma_lat_hinf_point)])
        gamma_lat_hinfs(i,j,k) = gamma_lat_hinf_point;
        K_lat_hinfs(:,:,i,j,k) = K_lat_hinf_point;
    end
    end
    end
    K_lat_hinf = pss(K_lat_hinfs,Domain);
    save K_lat_hinf.mat K_lat_hinf gamma_lat_hinfs SynTime_lat_hinfs
else
    disp('Lateral H_infinity Synthesis Skipped & Data Loaded')
    load('./result/K_lat_hinf.mat')
end

systemnames         = 'P_lat K_lat_hinf W_r_lat P_a_lat';
inputvar            = '[r{2}]';
outputvar           = '[P_lat ; K_lat_hinf]';
input_to_P_lat      = '[P_a_lat]';
input_to_P_a_lat    = '[K_lat_hinf]';
input_to_K_lat_hinf = '[W_r_lat(1)-P_lat(1) ; W_r_lat(2)-P_lat(4) ; P_lat(2) ; P_lat(3)]';
input_to_W_r_lat    = '[r]';
sysoutname          = 'CL_lat_hinf';
cleanupsysic        = 'yes'; 
sysic

% Lateral LPV synthesis
if isfile('./result/K_lat_lpv.mat') == 0
%     disp('Lateral LPV Synthess Started')
    datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss Z')
    tic
    [K_lat_lpv,gamma_lat_lpv,info_lat_lpv] = lpvsyn(G_lat,nmeas_lat,ncont_lat,opt_lpv);
    SynTime_lat_lpv = toc;
%     disp('Lateral LPV Synthess Finished')
%     disp(['Synthesis Time: ',num2str(SynTime_lat_lpv),' sec'])
%     datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss Z')
    save K_lat_lpv.mat K_lat_lpv gamma_lat_lpv info_lat_lpv SynTime_lat_lpv
else
    disp('Lateral LPV Synthesis Skipped & Data Loaded')
    load('./result/K_lat_lpv.mat')
end
% disp(['Min Gamma (Lateral Rate-Unbounded LPV) = ',num2str(info_lat_lpv.MinGamma)])
disp(['Gamma (Lateral Rate-Unbounded LPV) = ',num2str(gamma_lat_lpv)])
% disp(['Back-Off Factor (Lateral Rate-Unbounded LPV) = ',num2str((gamma_lat_lpv/info_lat_lpv.MinGamma-1)*100),' %'])

% Closed-loop interconnection with Lateral LPV controller and anti-aliasing filter
systemnames         = 'P_lat K_lat_lpv W_r_lat P_a_lat';
inputvar            = '[r{2}]';
outputvar           = '[P_lat ; K_lat_lpv]';
input_to_P_lat      = '[P_a_lat]';
input_to_P_a_lat    = '[K_lat_lpv]';
input_to_K_lat_lpv  = '[W_r_lat(1)-P_lat(1) ; W_r_lat(2)-P_lat(4) ; P_lat(2) ; P_lat(3)]';
input_to_W_r_lat    = '[r]';
sysoutname          = 'CL_lat_lpv';
cleanupsysic        = 'yes'; 
sysic

figure
step(CL_lat_hinf);
hold on, grid on
step(CL_lat_lpv);
legend('H\infty','LPV')
drawnow

%% Lateral rate-bounded LPV synthesis
% G_lat.Domain = DomainRB;
% if isfile('./result/K_lat_rblpv.mat') == 0
%     disp('Lateral RBLPV Synthess Finished')
%     datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss Z')
%     tic
%     [K_lat_rblpv,gamma_lat_rblpv,info_lat_rblpv] = lpvsyn(G_lat,7,2,Xb,Yb); % Elapsed time is 366 seconds with 4 basis functions
%     SynTime_lat_rblpv = toc;
%     disp('Lateral RBLPV Synthess Finished')
%     disp(['Synthesis Time: ',num2str(SynTime_lat_rblpv),' sec'])
%     datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss Z')
%     save K_lat_rblpv.mat K_lat_rblpv gamma_lat_rblpv info_lat_rblpv SynTime_lat_rblpv
% else
%     disp('Lateral Rate-Bounded LPV Synthesis Skipped & Data Loaded')
%     load('./result/K_lat_rblpv.mat')
% end
% G_lat.Domain = Domain;
% % disp(['Min Gamma (Lateral Rate-Bounded LPV) = ',num2str(info_lat_rblpv.MinGamma)])
% disp(['Gamma (Lateral Rate-Bounded LPV) = ',num2str(gamma_lat_rblpv)])
% % disp(['Back-Off Factor (Lateral Rate-Bounded LPV) = ',num2str((gamma_lat_rblpv/info_lat_rblpv.MinGamma-1)*100),' %'])

% P_lat.Domain = DomainRB;
% systemnames        = 'P_lat K_lat_rblpv D_ail UL_lat Ail Rud Ail_Delay Rud_Delay DL_lat AA_lat';
% inputvar           = '[bet_c{1};phi_c{1};Dist_ail{1}]';
% outputvar          = '[P_lat]';
% input_to_UL_lat    = '[K_lat_rblpv]';
% input_to_Ail       = '[UL_lat(1)]';
% input_to_Rud       = '[UL_lat(2)]';
% input_to_Ail_Delay = '[Ail+D_ail]';
% input_to_Rud_Delay = '[Rud]';
% input_to_P_lat     = '[Ail_Delay;Rud_Delay]';
% input_to_D_ail     = '[Dist_ail]';
% input_to_DL_lat    = '[AA_lat]';
% input_to_AA_lat    = '[P_lat]';
% input_to_K_lat_rblpv = '[bet_c;phi_c;P_lat]';
% sysoutname         = 'CL_lat_rblpv';
% cleanupsysic       = 'yes'; 
% sysic
% P_lat.Domain = Domain;

%% Nonlinear simulation
MWS               = init_design('GTM');
target            = struct('gamma',0,'tas',85); % start trim condition target [knot]
MWS.StatesInp(9)  = 0; % set altitude [ft]
MWS.bias.morphing = -12.5; % set morphing parameter
% MWS.bias.aileron = 0;
% MWS.bias.rudder = 0;
loadmws(MWS,'gtm_design'); % load MWS
set_param('gtm_design/Control/Controller','commented','on') % control off
set_param('gtm_design/Guidance/Heading Control','commented','on') % control off
set_param('gtm_design/Guidance/Windless','commented','on') % control off
set_param('gtm_design','MinStep','auto')
[MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose); % trim calculation
% MWS.StatesInp(4) = 1*d2r; % p [rad/s]
% MWS.StatesInp(5) = 1*d2r; % q [rad/s] 
% MWS.StatesInp(6) = 1*d2r; % r [rad/s]
% MWS.StatesInp(10) = 1*d2r; % phi [rad/s]
% MWS.StatesInp(11) = 1*d2r; % theta [rad/s]
% MWS.StatesInp(12) = 1*d2r; % psi [rad/s]
loadmws(MWS,'gtm_design'); % load MWS

set_param('gtm_design/Control/Controller','commented','off') % control on
set_param('gtm_design/Guidance/Heading Control','commented','off') % control on
set_param('gtm_design/Guidance/Windless','commented','off') % control off
set_param('gtm_design','MinStep','0.0049')
t_final = 250;
[tsim,xsim,ysim]=sim('gtm_design',[0 t_final]);