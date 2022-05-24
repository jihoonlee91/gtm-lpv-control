%% Basic setup
PitchSurf      = 'elev'; % pitch surface
verbose        = 1; % trim output display off
vabflag        = 1; % wind-axis linear state
use_all_inputs = 0; % control surface as groups
Ts             = 0; % continuous-time model
LPVControlOn   = 1; % 0 (hinf control) or 1 (lpv control)

% Parameter grid
tas_grid  = convvel([70 85 100],'kts','ft/s'); % [ft/s] (70~85 knot)
alt_grid  = [0 3000 6000]; % 700~1100 [ft]
mor_grid  = [-0.25 -0.125 0]; % between -0.25 and 0 []
num_tas   = numel(tas_grid); % number tas grid
num_alt   = numel(alt_grid); % number alt grid
num_mor   = numel(mor_grid); % number morphing grid

g = 32.17405; % gravitational acceleration [ft/s^2]
Coeff_throttle = [0.000033035538475  -0.008354385314499 1.191126728328792 31.518959847101669];
Coeff_thrust   = [0.000014847593013,  0, 0, 0.412662356277121];
T_max = polyval(Coeff_thrust,polyval(Coeff_throttle,100)); % Maximum thrust at sea level without ram drag
tas_RateBound = 5*g; % 2*T_max/MWS.w0 = 0.53g = 17 ft/s^2
alt_RateBound = 168; % convvel([100],'kts','ft/s')
mor_RateBound = 0.25; % 1 sec for full deployment

Domain = rgrid({'rho1_tas_param','rho2_alt_param','rho3_mor_param'},{tas_grid,alt_grid,mor_grid});
DomainRB = rgrid({'rho1_tas_param','rho2_alt_param','rho3_mor_param'},{tas_grid,alt_grid,mor_grid},[[-1,1]*tas_RateBound;[-1,1]*alt_RateBound;[-1,1]*mor_RateBound]);

s = tf('s');

%% Trim calculation
if isfile('./result/Trim.mat') == 0
    count = 0;
    for i = 1:num_tas
    for j = 1:num_alt
    for k = 1:num_mor
        tas = tas_grid(i); % current tas [ft/s]
        alt = alt_grid(j); % current alt [ft]
        mor = mor_grid(k); % current mor []
    
        target            = struct('gamma',0,'tas',convvel(tas,'ft/s','kts')); % target trim condition - straight and level flight, heading north [deg] X [knot]
        MWS.StatesInp(9)  = alt; % set altitude [ft]
        MWS.bias.morphing = mor*100; % set symmetric morphing parameter [%]
    
        loadmws(MWS,'gtm_design'); % load MWS
    
        [MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose); % trim calculation
        
        if (Err > 1e-3)
            error('TRIM FAIL')
        end
        count = count+1;
        disp(['Trim Done #',num2str(count)])
    
        % save trim results
        MWS_grid(i,j,k)      = MWS;
    
        tas_trim(i,j,k)      = convvel(Trimcond.tas,'kts','ft/s'); % [ft/s]
        alt_trim(i,j,k)      = Trimcond.altitude; % [ft]
        mor_trim(i,j,k)      = mor*100; % [%]
        
        alp_trim(i,j,k)      = Trimcond.alpha; % [deg]
        pitch_trim(i,j,k)    = Trimcond.pitch; % [deg]
        throttle_trim(i,j,k) = Trimcond.throttle; % [%]
        aileron_trim(i,j,k)  = Trimcond.aileron; % [deg]
        elevator_trim(i,j,k) = Trimcond.elevator; % [deg]
        rudder_trim(i,j,k)   = Trimcond.rudder; % [deg]
    
        clearvars sys_alp_trim sys_pitch_trim sys_throttle_trim sys_aileron_trim sys_elevator_trim sys_rudder_trim
        sys_alp_trim(:,:,i,j,k)      = ss(0,0,0,alp_trim(i,j,k)); % [deg]
        sys_pitch_trim(:,:,i,j,k)    = ss(0,0,0,pitch_trim(i,j,k)); % [deg]
        sys_throttle_trim(:,:,i,j,k) = ss(0,0,0,throttle_trim(i,j,k)); % [%]
        sys_aileron_trim(:,:,i,j,k)  = ss(0,0,0,aileron_trim(i,j,k)); % [deg]
        sys_elevator_trim(:,:,i,j,k) = ss(0,0,0,elevator_trim(i,j,k)); % [deg]
        sys_rudder_trim(:,:,i,j,k)   = ss(0,0,0,rudder_trim(i,j,k)); % [deg]
    end
    end
    end
    
    sch_alp_trim      = pss(sys_alp_trim,Domain); % [deg]
    sch_pitch_trim    = pss(sys_pitch_trim,Domain); % [deg]
    sch_throttle_trim = pss(sys_throttle_trim,Domain); % [%]
    sch_aileron_trim  = pss(sys_aileron_trim,Domain); % [deg]
    sch_elevator_trim = pss(sys_elevator_trim,Domain); % [deg]
    sch_rudder_trim   = pss(sys_rudder_trim,Domain); % [deg]
    
    save Trim.mat MWS_grid sch_alp_trim sch_pitch_trim sch_throttle_trim sch_aileron_trim sch_elevator_trim sch_rudder_trim
    disp('Trim Data Saved')
else
    load Trim.mat
    disp('Trim Data Loaded')
end

%% Linearization
if isfile('./result/Linearization.mat') == 0
    clearvars GTM_lon GTM_lat
    count = 0;
    for i = 1:num_tas
    for j = 1:num_alt
    for k = 1:num_mor
        % linearization
        [~,londyn,latdyn]=linmodel(MWS_grid(i,j,k),vabflag,use_all_inputs,Ts);
        count = count+1;
        disp(['Linearization Done #',num2str(count)])
        
        % Redefine input name
        set(latdyn,'Inputname',{'A Morphing','Rudder','L Spoiler','R Spoiler'});
        
        % Units
        % Lon state: tas [ft/s], alpha [rad], q [rad/s], theta [rad]
        % Lon input: Elevator [deg], L Spoiler [deg], R Spoiler [deg], L Throttle [%], R Throttle [%]
        % Lat state: beta [rad], p [rad/s], r [rad/s], phi [rad]
        % Lat input: A Morphing [deg], Rudder [deg], L Spoiler [deg], R Spoiler [deg]
    
        % unit change
        londyn.B(:,1:3) = londyn.B(:,1:3)*180/pi; % Elevator, L/R Spoilers [deg] -> [rad]
        londyn.B(:,4:5) = londyn.B(:,4:5)*100; % L/R Throttles [%] -> []
        latdyn.B(:,1)   = latdyn.B(:,1)*100; % A Morphing [%] -> []
        latdyn.B(:,2:4) = latdyn.B(:,2:4)*180/pi; % Rudder, L/R Spoilers [deg] -> [rad]
        
        % New units:
        % Lon state: tas [ft/s], alpha [rad], q [rad/s], theta [rad]
        % Lon input: Elevator [rad], L Spoiler [rad], R Spoiler [rad], L Throttle [], R Throttle []
        % Lat state: beta [rad], p [rad/s], r [rad/s], phi [rad]
        % Lat input: Aileron [rad], Rudder [rad], L Spoiler [rad], R Spoiler [rad]
        
        % Trim condition
        V0 = convvel(tas_trim(i),'kts','ft/s'); % trim speed [ft/s]
        alp_0 = deg2rad(Trimcond.alpha); % trim angle of attack [rad]
        gam_0 = deg2rad(Trimcond.pitch-Trimcond.alpha); % trim pitch angle [rad]
        
        % longitudinal dimensional derivatives
        X_V = londyn.A(1,1); % [ft/s^2] / [ft/s]
        X_alp = londyn.A(1,2); % [ft/s^2] / [rad]
        Z_V = londyn.A(2,1)*V0; % [rad/s]
        Z_alp  = londyn.A(2,2)*V0; % [rad/s] X [ft/s] / [rad]
        Z_q = (londyn.A(2,3)-1)*V0; % [ft/s]
        M_V = londyn.A(3,1); % [rad/s^2] / [ft/s]
        M_alp = londyn.A(3,2); % [rad/s^2] / [rad]
        M_q = londyn.A(3,3); % [rad/s^2] / [rad/s]
        X_dele = londyn.B(1,2); % [ft/s^2] / [rad]
        Z_dele = londyn.B(2,1)*V0; % [rad/s] X [ft/s] / [rad]
        M_dele = londyn.B(3,1); % [rad/s] / [rad]
        
        % longitudinal dynamics truncation and output augmentation (extract short-period mode and add acceleration output)
        Alon = [Z_alp/V0 1+Z_q/V0;
                M_alp M_q];
        Blon = [Z_dele/V0;
                M_dele];
        Clon = [1 0;
                0 1;
                Z_alp 0];
        Dlon = [0;
                0;
                Z_dele];
        londyn_aug = ss(Alon,Blon,Clon,Dlon);
        set(londyn_aug,'Statename',{'alpha','q',},...
                       'Inputname',{'dele'},...
                       'Outputname',{'alpha','q','Az'})
    
        % lateral dimensional derivatives
        Y_bet = latdyn.A(1,1)*V0;
        Y_p = latdyn.A(1,2)*V0;
        Y_r = latdyn.A(1,3)*V0;
        Y_dela = latdyn.B(1,1)*V0;
        Y_delr = latdyn.B(1,2)*V0;
    
        % Lateral dynamics augmentation
        Alat = latdyn.A;
        Blat = latdyn.B(:,1:2);
        Clat = [latdyn.C;
                Y_bet Y_p Y_r 0];
        Dlat = [latdyn.D(:,1:2);
                Y_dela Y_delr];
        latdyn_aug = ss(Alat,Blat,Clat,Dlat);
        set(latdyn_aug,'Statename',{'beta','p','r','phi'},...
                       'Inputname',{'dela','delr'},...
                       'Outputname',{'beta','p','r','phi','Ay'})
        
        % stacking
        GTM_lon(:,:,i,j,k) = londyn_aug;
        GTM_lat(:,:,i,j,k) = latdyn_aug;
    end
    end
    end
    
    P_lon = pss(GTM_lon,Domain);
    P_lat = pss(GTM_lat,Domain);

    save Linearization.mat P_lon P_lat
    disp('Linearization Data Saved')
else
    load Linearization.mat
    disp('Linearization Data Loaded')
end

% figure
% subplot(121), bode(P_lon), grid on
% subplot(122), bode(P_lat), grid on

%% Component modeling (physical)
% Actuator dynamics
w_ele = 2*pi*5; % elevator frequency [Hz], W_ele = w_ele/(s+w_ele)
w_ail = 2*pi*0.15; % morphing actuator frequency [Hz], W_ail = w_ail/(s+w_ail)
w_rud = 2*pi*5; % rudder frequency [Hz], W_rud = w_rud/(s+w_rud)
Eng = tf(ss([-1.336,-0.7314;1,0;],[1;0],[-0.1474   0.7314],0));
Ail = w_ail/(s+w_ail);
Ele = w_ele/(s+w_ele);
Rud = w_rud/(s+w_rud);

% Time delay pade approximation
Pade5 = pade(exp(-5e-3*s),1);
Pade10 = pade(exp(-10e-3*s),1);
Pade15 = pade(exp(-15e-3*s),1);
Pade30 = pade(exp(-30e-3*s),1);

% Anti-Aliasing Filter
AA = 1/(1/2/pi/16*s+1);

%% Component modeling (longitudinal performance)
% Ideal model
omega_An = 8;
zeta_An = 1;
Ideal_An = (omega_An^2)/(s^2+2*zeta_An*omega_An*s+omega_An^2);

% Weight function design
Model_An  = Ideal_An*Ele*Pade30;
Perf_An   = 100*tf([0.1,5.973],[1,0.176]);
W_elev    = (0.8*s+42.91)/(s+119.2);
N_alp     = 0.031*pi/180*(s+10)/(s+100); % 0.031 deg/s
N_q       = 0.3857*pi/180*(s+10)/(s+100); % 0.3857 deg/s
N_An      = 0.0082*g*(s+10)/(s+100); % 0.0082 g

UL = Pade5;
DL = [Pade15 0 0;
      0 Pade15 0;
      0 0 Pade15];
Ele_Delay = Pade10;
% AA_lon = [AA 0 0;
%           0 AA 0;
%           0 0 AA];
AA_lon = eye(3);
N = [N_alp 0 0;
     0 N_q 0;
     0 0 N_An];

%% Longitudinal LPV modeling
% control interconnection
systemnames        = 'P_lon N Model_An Perf_An W_elev Ele UL Ele_Delay DL';
inputvar           = '[An_c{1};Noise{3};u{1}]';
outputvar          = '[Perf_An;W_elev;An_c;DL]';
input_to_UL        = '[u]';
input_to_Ele       = '[UL]';
input_to_Ele_Delay = '[Ele]';
input_to_P_lon     = '[Ele_Delay]';
input_to_N         = '[Noise]';
input_to_DL        = '[P_lon+N]';
input_to_Model_An  = '[An_c]';
input_to_Perf_An   = '[Model_An-P_lon(3)]';
input_to_W_elev    = '[u]';
sysoutname         = 'G_lon';
cleanupsysic       = 'yes'; 
sysic

%% Longitudinal LPV synthesis
G_lon.Domain = Domain;
if isfile('./result/K_lon_lpv.mat') == 0
    tic
    [K_lon_lpv,gamma_lon_lpv,info_lon_lpv] = lpvsyn(G_lon,4,1);
    SynTime_lon_lpv = toc;
    save K_lon_lpv.mat K_lon_lpv gamma_lon_lpv info_lon_lpv SynTime_lon_lpv
else
    disp('Longitudinal LPV Synthesis Skipped & Data Loaded')
    load K_lon_lpv.mat
end
% disp(['Min Gamma (Longitudinal Rate-Unbounded LPV) = ',num2str(info_lon_lpv.MinGamma)])
disp(['Gamma (Longitudinal Rate-Unbounded LPV) = ',num2str(gamma_lon_lpv)])
% disp(['Back-Off Factor (Longitudinal Rate-Unbounded LPV) = ',num2str((gamma_lon_lpv/info_lon_lpv.MinGamma-1)*100),' %'])

%% Longitudinal H_infinity synthesis
if isfile('./result/K_lon_hinf.mat') == 0
    clearvars K_lon_hinfs gamma_lon_hinfs
    for i = 1:num_tas
    for j = 1:num_alt
    for k = 1:num_mor 
        G_lon_LTI = G_lon.value('rho1_tas_param',tas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
        tic
        [K_lon_hinf_point,~,gamma_lon_hinf_point] = hinfsyn(G_lon_LTI,4,1);
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
    load K_lon_hinf.mat
end

%% Closed-loop interconnection with Longitudinal H_infinity controller and anti-aliasing filter
systemnames         = 'P_lon K_lon_hinf Ele UL Ele_Delay AA_lon DL';
inputvar            = '[An_c{1}]';
outputvar           = '[DL]';
input_to_UL         = '[K_lon_hinf]';
input_to_Ele        = '[UL]';
input_to_Ele_Delay  = '[Ele]';
input_to_P_lon      = '[Ele_Delay]';
input_to_AA_lon     = '[P_lon]';
input_to_DL         = '[AA_lon]';
input_to_K_lon_hinf = '[An_c;DL]';
sysoutname          = 'CL_lon_hinf';
cleanupsysic        = 'yes'; 
sysic

%% Closed-loop comparison
figure
step(CL_lon_lpv)
step(CL_lon_hinf)
grid on
