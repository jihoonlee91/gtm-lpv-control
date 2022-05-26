%% Initialization
close all

% clear all
% Load any data for initialization
addpath(genpath('./LPVToolsV1.0/'));
% load aTrim.mat
% load aLinearization.mat
% load aK_lon_lpv.mat
% load aK_lon_hinf.mat
% load aK_lat_lpv.mat
% load aK_lat_hinf.mat
setup
% MWS = grabmws('gtm_design');

PitchSurf      = 'elev'; % pitch surface
verbose        = 1; % trim output display off
vabflag        = 1; % wind-axis linear state
use_all_inputs = 0; % control surface as groups
Ts             = 0; % continuous-time model

opt  = lpvsynOptions('BackOffFactor',1.02);
opts = hinfsynOptions('Method','RIC');

%% Basic setup
 
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
tas_RateBound = 10; % 2*T_max/MWS.w0 = 0.53g = 17 ft/s^2
alt_RateBound = 20; % convvel([100],'kts','ft/s')
mor_RateBound = 0.025; % 10 sec for full deployment

Domain = rgrid({'rho1_tas_param','rho2_alt_param','rho3_mor_param'},{tas_grid,alt_grid,mor_grid});
DomainRB = rgrid({'rho1_tas_param','rho2_alt_param','rho3_mor_param'},{tas_grid,alt_grid,mor_grid},[[-1,1]*tas_RateBound;[-1,1]*alt_RateBound;[-1,1]*mor_RateBound]);

s = tf('s');
w=logspace(-3,3,50);

%% Trim calculation
if isfile('./result/Trim.mat') == 0
    clearvars MWS_grid tas_trim alt_trim mor_trim alp_trim pitch_trim throttle_trim aileron_trim elevator_trim rudder_trim
    clearvars sys_alp_trim sys_pitch_trim sys_throttle_trim sys_aileron_trim sys_elevator_trim sys_rudder_trim
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

        sys_tas_trim(:,:,i,j,k)      = ss(0,0,0,tas_trim(i,j,k)); % [deg]
        sys_alt_trim(:,:,i,j,k)      = ss(0,0,0,alt_trim(i,j,k)); % [deg]
        sys_mor_trim(:,:,i,j,k)      = ss(0,0,0,mor_trim(i,j,k)); % [deg]

        sys_alp_trim(:,:,i,j,k)      = ss(0,0,0,alp_trim(i,j,k)); % [deg]
        sys_pitch_trim(:,:,i,j,k)    = ss(0,0,0,pitch_trim(i,j,k)); % [deg]
        sys_throttle_trim(:,:,i,j,k) = ss(0,0,0,throttle_trim(i,j,k)); % [%]
        sys_aileron_trim(:,:,i,j,k)  = ss(0,0,0,aileron_trim(i,j,k)); % [deg]
        sys_elevator_trim(:,:,i,j,k) = ss(0,0,0,elevator_trim(i,j,k)); % [deg]
        sys_rudder_trim(:,:,i,j,k)   = ss(0,0,0,rudder_trim(i,j,k)); % [deg]
    end
    end
    end
    
    sch_tas_trim      = pss(sys_tas_trim,Domain); % [deg]
    sch_alt_trim      = pss(sys_alt_trim,Domain); % [deg]
    sch_mor_trim      = pss(sys_mor_trim,Domain); % [deg]

    sch_alp_trim      = pss(sys_alp_trim,Domain); % [deg]
    sch_pitch_trim    = pss(sys_pitch_trim,Domain); % [deg]
    sch_throttle_trim = pss(sys_throttle_trim,Domain); % [%]
    sch_aileron_trim  = pss(sys_aileron_trim,Domain); % [deg]
    sch_elevator_trim = pss(sys_elevator_trim,Domain); % [deg]
    sch_rudder_trim   = pss(sys_rudder_trim,Domain); % [deg]
    
    save Trim.mat MWS_grid sch_tas_trim sch_alt_trim sch_mor_trim sch_alp_trim sch_pitch_trim sch_throttle_trim sch_aileron_trim sch_elevator_trim sch_rudder_trim tas_trim alt_trim mor_trim
    disp('Trim Data Saved')
else
    load Trim.mat
    disp('Trim Data Loaded')
end

%% Linearization
w_ele = 2*pi*5; % elevator frequency [Hz], W_ele = w_ele/(s+w_ele)
w_ail = 2*pi*2; % morphing actuator frequency [Hz], W_ail = w_ail/(s+w_ail)
w_rud = 2*pi*5; % rudder frequency [Hz], W_rud = w_rud/(s+w_rud)

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
        V0 = tas_grid(i); % trim speed [ft/s]
        alp_0 = deg2rad(sch_alp_trim.value('rho1_tas_param',tas_grid(i),'rho2_alt_param',alt_grid(i),'rho3_mor_param',mor_grid(i)).D); % trim angle of attack [rad]
        theta_0 = sch_pitch_trim.value('rho1_tas_param',tas_grid(i),'rho2_alt_param',alt_grid(i),'rho3_mor_param',mor_grid(i)).D;
        gam_0 = deg2rad(theta_0-alp_0); % trim pitch angle [rad]
        
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

        % longitudinal dynamics truncation and output augmentation (error state)
%         Alon = [0 1 0 0
%                 0 Z_alp/V0 Z_alp Z_dele;
%                 0 M_alp/Z_alp M_q M_dele-M_alp*Z_dele/Z_alp
%                 0 0 0 -w_ele];
%         Blon = [0
%                 0;
%                 0;
%                 w_ele];
%         Clon = eye(4);
%         Dlon = [0;
%                 0;
%                 0
%                 0];
%         londyn_aug = ss(Alon,Blon,Clon,Dlon);
%         set(londyn_aug,'Statename',{'e','Az','q','dele'},...
%                        'Inputname',{'delec'},...
%                        'Outputname',{'e','Az','q','dele'})

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

    save Linearization.mat P_lon P_lat GTM_lon GTM_lat
    disp('Linearization Data Saved')
else
    load Linearization.mat
    disp('Linearization Data Loaded')
end

%% Component modeling (physical)
% Actuator dynamics
Eng = tf(ss([-1.336,-0.7314;1,0;],[1;0],[-0.1474   0.7314],0));
Ail = w_ail/(s+w_ail);
Ele = w_ele/(s+w_ele);
Rud = w_rud/(s+w_rud);

% Time delay pade approximation
Pade5 = pade(exp(-5e-3*s),1);
Pade10 = pade(exp(-10e-3*s),1);
Pade15 = pade(exp(-15e-3*s),1);
Pade30 = pade(exp(-30e-3*s),1);
Pade40 = pade(exp(-40e-3*s),1);

% Anti-Aliasing Filter
AA = 1/(1/2/pi/16*s+1);

% figure
% bode(Eng,'k-',Ail,'r--',Ele,'g:',Rud,'b-.',w),title(''), grid on
% legend('Throttle Lag','Asymmetric Morphing','Elevator','Rudder','location','best')
% 
% figure
% bode(AA),title(''), grid on
% title('Anti-Aliasing Filter')

%% Component modeling (longitudinal performance)
% Ideal model
omega_Az = 16;
zeta_Az = 0.7;
Ideal_Az = (omega_Az^2)/(s^2+2*zeta_Az*omega_Az*s+omega_Az^2);

% Weight function design
Model_Az  = Ideal_Az*Ele*Pade30;
Perf_Az   = tf([1,39.73],[1,0.176]);
W_elev    = 10*(s+6)/(s+120);
N_alp     = 0.031*pi/180; % 0.031 deg/s
N_q       = 0.3857*pi/180; % 0.3857 deg/s
N_Az      = 0.0082*g; % 0.0082 g

UL = Pade5;
DL = [Pade15 0 0;
      0 Pade15 0;
      0 0 Pade15];
Ele_Delay = Pade10;
AA_lon = [AA 0 0;
          0 AA 0;
          0 0 AA];
N = [N_alp 0 0;
     0 N_q 0;
     0 0 N_Az];
D = 0.01;

Int = 5/(s+0.001);

% figure
% bode(Ideal_Az,Model_Az),title(''), grid on
% h = legend('$Id_{Az}$','$M_{Az}$','Interpreter','latex');
% set(h,'string',{'$Id_{Az}$','$M_{Az}$'})

% figure
% bode(Perf_Az,W_elev),title(''), grid on
% h = legend('$P_{Az}$','$W_{\delta e}$','Interpreter','latex');
% set(h,'string',{'$P_{Az}$','$W_{\delta e}$'})

%% Longitudinal LPV modeling
% control interconnection
systemnames        = 'P_lon N Model_Az W_elev Ele Int Perf_Az Pade30';
inputvar           = '[Az_c{1};Noise{3};u{1}]';
% outputvar          = '[W_elev;Perf_Az;Int;Az_c;P_lon+N]';
outputvar          = '[W_elev;Int;Az_c;P_lon+N]';
input_to_Ele       = '[u]';
input_to_Pade30    = '[Ele]';
input_to_P_lon     = '[Pade30]';
input_to_N         = '[Noise]';
input_to_Model_Az  = '[Az_c]';
input_to_W_elev    = '[u]';
input_to_Perf_Az   = '[Model_Az-P_lon(3)]';
input_to_Int       = '[Model_Az-P_lon(3)]';
sysoutname         = 'G_lon';
cleanupsysic       = 'yes'; 
sysic

%% Longitudinal H_infinity synthesis
if isfile('./result/K_lon_hinf.mat') == 0
    clearvars K_lon_hinfs gamma_lon_hinfs
    for i = 1:num_tas
    for j = 1:num_alt
    for k = 1:num_mor 
        G_lon_LTI = G_lon.value('rho1_tas_param',tas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
        tic
        [K_lon_hinf_point,CL,gamma_lon_hinf_point] = hinfsyn(G_lon_LTI,4,1,opts);
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

%% Closed-loop interconnection with Longitudinal H_infinity controller and anti-aliasing filter
systemnames         = 'P_lon K_lon_hinf Ele UL Ele_Delay AA_lon DL';
inputvar            = '[Az_c{1}]';
outputvar           = '[P_lon(3)]';
input_to_UL         = '[K_lon_hinf]';
input_to_Ele        = '[UL]';
input_to_Ele_Delay  = '[Ele]';
input_to_P_lon      = '[Ele_Delay]';
input_to_AA_lon     = '[P_lon]';
input_to_DL         = '[AA_lon]';
input_to_K_lon_hinf = '[Az_c;DL]';
sysoutname          = 'CL_lon_hinf';
cleanupsysic        = 'yes'; 
sysic

%% Longitudinal LPV synthesis
if isfile('./result/K_lon_lpv.mat') == 0
    disp('Longitudinal LPV Synthess Started')
    datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss Z')
    tic
    [K_lon_lpv,gamma_lon_lpv,info_lon_lpv] = lpvsyn(G_lon,4,1,opt);
    SynTime_lon_lpv = toc;
    disp('Longitudinal LPV Synthess Finished')
    disp(['Synthesis Time: ',num2str(SynTime_lon_lpv),' sec'])
    datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss Z')
    save K_lon_lpv.mat K_lon_lpv gamma_lon_lpv info_lon_lpv SynTime_lon_lpv
else
    disp('Longitudinal LPV Synthesis Skipped & Data Loaded')
    load('./result/K_lon_lpv.mat')
end
% disp(['Min Gamma (Longitudinal Rate-Unbounded LPV) = ',num2str(info_lon_lpv.MinGamma)])
disp(['Gamma (Longitudinal Rate-Unbounded LPV) = ',num2str(gamma_lon_lpv)])
% disp(['Back-Off Factor (Longitudinal Rate-Unbounded LPV) = ',num2str((gamma_lon_lpv/info_lon_lpv.MinGamma-1)*100),' %'])

%% Closed-loop interconnection with Longitudinal LPV controller and anti-aliasing filter
systemnames         = 'P_lon K_lon_lpv Ele UL Ele_Delay AA_lon DL';
inputvar            = '[Az_c{1}]';
outputvar           = '[P_lon(3)]';
input_to_UL         = '[K_lon_lpv]';
input_to_Ele        = '[UL]';
input_to_Ele_Delay  = '[Ele]';
input_to_P_lon      = '[Ele_Delay]';
input_to_AA_lon     = '[P_lon]';
input_to_DL         = '[AA_lon]';
input_to_K_lon_lpv  = '[Az_c;DL]';
sysoutname          = 'CL_lon_lpv';
cleanupsysic        = 'yes'; 
sysic

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
% % 
% G_lon.Domain = DomainRB;
% if isfile('./result/K_lon_rblpv.mat') == 0
%     disp('Longitudinal RBLPV Synthess Finished')
%     datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss Z')
%     tic
%     [K_lon_rblpv,gamma_lon_rblpv,info_lon_rblpv] = lpvsyn(G_lon,4,1,Xb,Yb); % Elapsed time is 366 seconds with 4 basis functions
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

%% Closed-loop interconnection with Longitudinal Rate-Bounded LPV controller and anti-aliasing filter
% P_lon.Domain = DomainRB;
% systemnames         = 'P_lon K_lon_rblpv Ele UL Ele_Delay AA_lon DL Int';
% inputvar            = '[Az_c{1}]';
% outputvar           = '[P_lon(3)]';
% input_to_UL         = '[K_lon_rblpv]';
% input_to_Ele        = '[UL]';
% input_to_Ele_Delay  = '[Ele]';
% input_to_P_lon      = '[Ele_Delay]';
% input_to_AA_lon     = '[P_lon]';
% input_to_DL         = '[AA_lon]';
% input_to_K_lon_rblpv= '[Az_c;DL]';
% sysoutname          = 'CL_lon_rblpv';
% cleanupsysic        = 'yes'; 
% sysic
% P_lon.Domain = Domain;

%% Closed-loop comparison
% figure
% step(CL_lon_lpv,10), hold on
% step(CL_lon_hinf,10)
% grid on
% legend('LPV','H\infty')
% drawnow

%% Component modeling (lateral performance)
% Ideal model
omega_beta = 3;
% zeta_beta = .9;

omega_phi = 10;
zeta_phi = .7;

Ideal_bet = tf(omega_beta,[1 omega_beta]);
Ideal_phi = tf(omega_phi^2,[1 2*zeta_phi*omega_phi omega_phi^2]);

% Weight function
Model_bet = Ideal_bet;
Model_phi = Ideal_phi;

Perf_bet = 0.15*tf([1,14],[1,0.4]);
Perf_phi = 0.1*tf([1,60],[1,2]);
% Perf_bet = tf([0.1,6],[1,0.178]);
% Perf_phi = tf([0.1,6],[1,0.3]);

W_ail = 1*(s+10)/(s+60);
W_rud = 1*(s+20)/(s+120);
% W_ail = (0.8*s+43)/(s+120);
% W_rud = (0.8*s+43)/(s+120);

Int_bet = .1/(s+0.001);
Int_phi = .1/(s+0.001);

D_ail = 0.01*tf([1,w_ail*10],[1,w_ail])*0;

N_bet = 0.0033*pi/180; % 0.033 deg
N_p = 0.0522*pi/180; % 0.522 deg/s
N_r = 0.0333*pi/180; % 0.333 deg/s
N_phi = 0.002*pi/180; % 0.02 deg
N_Ay = 0.00082*g; % 0.0082 g

UL_lat = [Pade5 0;
          0 Pade5];
DL_lat = [Pade15 0 0 0 0;
          0 Pade15 0 0 0;
          0 0 Pade15 0 0;
          0 0 0 Pade15 0;
          0 0 0 0 Pade15];
Ail_Delay = Pade10;
Rud_Delay = Pade10;
AA_lat = [AA 0 0 0 0;
          0 AA 0 0 0;
          0 0 AA 0 0;
          0 0 0 AA 0;
          0 0 0 0 AA];
N_lat = [N_bet 0 0 0 0;
         0 N_p 0 0 0;
         0 0 N_r 0 0;
         0 0 0 N_phi 0;
         0 0 0 0 N_Ay];

Phi_Delay = [1 0 0 0 0;
             0 1 0 0 0;
             0 0 1 0 0;
             0 0 0 Pade40 0;
             0 0 0 0 1];

AilD = Pade30;
RudD = Pade30;

figure
bode(Perf_bet,Perf_phi,W_ail,W_rud,D_ail),title(''), grid on
h = legend('$P_{\beta}$','$P_{\phi}$','$W_{\eta a}$','$W_{\delta r}$','$D_{\eta a}$','Interpreter','latex');
set(h,'string',{'$P_{\beta}$','$P_{\phi}$','$W_{\eta a}$','$W_{\delta r}$','$D_{\eta a}$'})

drawnow

%% Lateral LPV modeling
systemnames        = 'P_lat N_lat Model_bet Model_phi W_ail W_rud Ail Rud AilD RudD Int_bet Int_phi Perf_bet Perf_phi Phi_Delay';
inputvar           = '[bet_c{1};phi_c{1};Noise{5};u{2}]';
% outputvar          = '[W_ail;W_rud;Perf_bet;Perf_phi;Int_bet;Int_phi;bet_c;phi_c;Phi_Delay]';
outputvar          = '[W_ail;W_rud;Int_bet;Int_phi;bet_c;phi_c;Phi_Delay]';
input_to_Perf_bet  = '[Model_bet-P_lat(1)]';
input_to_Perf_phi  = '[Model_phi-P_lat(4)]';
input_to_Int_bet   = '[Model_bet-P_lat(1)]';
input_to_Int_phi   = '[Model_phi-P_lat(4)]';
input_to_Ail       = '[u(1)]';
input_to_Rud       = '[u(2)]';
input_to_AilD      = '[Ail]';
input_to_RudD      = '[Rud]';
input_to_P_lat     = '[AilD;RudD]';
input_to_N_lat     = '[Noise]';
input_to_Phi_Delay = '[P_lat+N_lat]';
input_to_Model_bet = '[bet_c]';
input_to_Model_phi = '[phi_c]';
input_to_W_ail     = '[u(1)]';
input_to_W_rud     = '[u(2)]';
sysoutname         = 'G_lat';
cleanupsysic       = 'yes'; 
sysic

%% Lateral H_infinity synthesis
if isfile('./result/K_lat_hinf.mat') == 0
    clearvars K_lat_hinfs gamma_lat_hinfs
    for i = 1:num_tas
    for j = 1:num_alt
    for k = 1:num_mor 
        G_lat_LTI = G_lat.value('rho1_tas_param',tas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
        tic
        [K_lat_hinf_point,~,gamma_lat_hinf_point] = hinfsyn(G_lat_LTI,7,2,opts);
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

%% Closed-loop interconnection with Lateral H_infinity controller and anti-aliasing filter
systemnames        = 'P_lat K_lat_hinf UL_lat Ail Rud Ail_Delay Rud_Delay DL_lat AA_lat Phi_Delay';
inputvar           = '[bet_c{1};phi_c{1}]';
outputvar          = '[P_lat(1);P_lat(4)]';
input_to_UL_lat    = '[K_lat_hinf]';
input_to_Ail       = '[UL_lat(1)]';
input_to_Rud       = '[UL_lat(2)]';
input_to_Ail_Delay = '[Ail]';
input_to_Rud_Delay = '[Rud]';
input_to_P_lat     = '[Ail_Delay;Rud_Delay]';
input_to_DL_lat    = '[AA_lat]';
input_to_Phi_Delay = '[P_lat]';
input_to_AA_lat    = '[Phi_Delay]';
input_to_K_lat_hinf= '[bet_c;phi_c;DL_lat]';
sysoutname         = 'CL_lat_hinf';
cleanupsysic       = 'yes'; 
sysic

%% Lateral LPV synthesis
if isfile('./result/K_lat_lpv.mat') == 0
    disp('Lateral LPV Synthess Started')
    datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss Z')
    tic
    [K_lat_lpv,gamma_lat_lpv,info_lat_lpv] = lpvsyn(G_lat,7,2,opt);
    SynTime_lat_lpv = toc;
    disp('Lateral LPV Synthess Finished')
    disp(['Synthesis Time: ',num2str(SynTime_lat_lpv),' sec'])
    datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss Z')
    save K_lat_lpv.mat K_lat_lpv gamma_lat_lpv info_lat_lpv SynTime_lat_lpv
else
    disp('Lateral LPV Synthesis Skipped & Data Loaded')
    load('./result/K_lat_lpv.mat')
end
disp(['Min Gamma (Lateral Rate-Unbounded LPV) = ',num2str(info_lat_lpv.MinGamma)])
disp(['Gamma (Lateral Rate-Unbounded LPV) = ',num2str(gamma_lat_lpv)])
disp(['Back-Off Factor (Lateral Rate-Unbounded LPV) = ',num2str((gamma_lat_lpv/info_lat_lpv.MinGamma-1)*100),' %'])

%% Closed-loop interconnection with Lateral LPV controller and anti-aliasing filter
systemnames        = 'P_lat K_lat_lpv UL_lat Ail Rud Ail_Delay Rud_Delay DL_lat AA_lat Phi_Delay';
inputvar           = '[bet_c{1};phi_c{1}]';
outputvar          = '[P_lat(1);P_lat(4)]';
input_to_UL_lat    = '[K_lat_lpv]';
input_to_Ail       = '[UL_lat(1)]';
input_to_Rud       = '[UL_lat(2)]';
input_to_Ail_Delay = '[Ail]';
input_to_Rud_Delay = '[Rud]';
input_to_P_lat     = '[Ail_Delay;Rud_Delay]';
input_to_DL_lat    = '[AA_lat]';
input_to_Phi_Delay = '[P_lat]';
input_to_AA_lat    = '[Phi_Delay]';
input_to_K_lat_lpv= '[bet_c;phi_c;DL_lat]';
sysoutname         = 'CL_lat_lpv';
cleanupsysic       = 'yes'; 
sysic

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

%% Closed-loop interconnection with Lateral Rate-Bounded LPV controller and anti-aliasing filter
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

%% Closed-loop comparison
% figure
% step(CL_lat_lpv,10), hold on
% step(CL_lat_hinf,10)
% grid on

%%
% MWS               = init_design('GTM');
% target            = struct('gamma',0,'tas',85); % target trim condition [knot]
% MWS.StatesInp(9)  = 3000; % set altitude [ft]
% MWS.bias.morphing = -12.5; % set morphing parameter
% loadmws(MWS,'gtm_design'); % load MWS
% [MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose); % trim calculation
% loadmws(MWS,'gtm_design'); % load MWS

%%
[tsim,xsim,ysim]=sim('gtm_design',[0 10]);
