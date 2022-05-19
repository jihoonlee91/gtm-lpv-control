%% Initialization
close all
clear all
clc

setup
addpath(genpath('./LPVToolsV1.0/'));

MWS = grabmws('gtm_design');

%% Setting
PitchSurf      = 'elev'; % pitch surface
verbose        = 1; % trim output display off
vabflag        = 1; % wind-axis linear state
use_all_inputs = 0; % control surface as groups
Ts             = 0; % continuous-time model
LPVControlOn   = 0; % 0 (hinf control) or 1 (lpv control)
RateBoundOn    = 0; % 0 (rate-unbounded) or 1 (rate-bounded)

% Parameter grid
tas_grid  = [80]; % 70~85 [knot]
alt_grid  = [1000]; % 700~1100 [ft]
mor_grid  = [-12.5]; % between -25 and 0 [%]
num_tas   = numel(tas_grid); % number tas grid
num_alt   = numel(alt_grid); % number alt grid
num_mor   = numel(mor_grid); % number morphing grid
tas_param = pgrid('tas_param',tas_grid);
alt_param = pgrid('alt_param',alt_grid);
mor_param = pgrid('mor_param',mor_grid);

% Rate bounds
if RateBoundOn == 1
    tas_rbound = 60; % [kts/s]
    alt_rbound = 25; % [ft/s]
    mor_rbound = 10; % [%/s]
    tas_param.RateBounds = [-1,1]*tas_rbound;
    alt_param.RateBounds = [-1,1]*alt_rbound;
    mor_param.RateBounds = [-1,1]*mor_rbound;
end

% dummy input for trimming and linearization
Pilot_lon = 0;
Pilot_lat = 0;
Pilot_ped = 0;

s = tf('s');
g = 32.17405; % gravitational acceleration [ft/s^2]

%% Trimming
for i = 1:num_tas
for j = 1:num_alt
for k = 1:num_mor
    ind = sub2ind([num_tas num_alt num_mor],i,j,k);

    tas = tas_grid(i); % current tas
    alt = alt_grid(j); % current alt
    mor = mor_grid(k); % current mor

    target            = struct('gamma',0,'tas',tas); % target trim condition
    MWS.StatesInp(9)  = alt; % set altitude
    MWS.bias.morphing = mor; % set morphing parameter

    loadmws(MWS,'gtm_design'); % load MWS

    [MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose); % trim calculation

    % save trim results
    MWS_grid(i,j,k)      = MWS;
    tas_trim(i,j,k)      = Trimcond.tas;
    alt_trim(i,j,k)      = Trimcond.altitude;
    mor_trim(i,j,k)      = mor;
    alp_trim(i,j,k)      = Trimcond.alpha;
    pitch_trim(i,j,k)    = Trimcond.pitch;
    throttle_trim(i,j,k) = Trimcond.throttle;
    aileron_trim(i,j,k)  = Trimcond.aileron;
    elevator_trim(i,j,k) = Trimcond.elevator;
    rudder_trim(i,j,k)   = Trimcond.rudder;

    if (Err > 1e-3)
        error('TRIM FAIL')
    end
    disp('-------------------------------------------------------------------')
end
end
end

%% Linearization
w_ele = 2*pi*5; % elevator frequency [Hz], W_ele = w_ele/(s+w_ele)
w_ail = 2*pi*5; % morphing actuator frequency [Hz], W_ail = w_ail/(s+w_ail)
w_rud = 2*pi*5; % rudder frequency [Hz], W_rud = w_rud/(s+w_rud)

clearvars GTM_lon GTM_lat
for i = 1:num_tas
for j = 1:num_alt
for k = 1:num_mor    
    % linearization
    [~,londyn,latdyn]=linmodel(MWS_grid(i,j,k),vabflag,use_all_inputs,Ts);

    % Name setting
    set(latdyn,'Inputname',{'A Morphing','Rudder','L Spoiler','R Spoiler'});
    
    % tas [ft/s], alpha [rad], q [rad/s], theta [rad]
    % Elevator [deg], L Spoiler [deg], R Spoiler [deg]
    % L Throttle [%], R Throttle [%]
    % beta [rad], p [rad/s], r [rad/s], phi [rad]
    % A Morphing [deg], Rudder [deg], L Spoiler [deg], R Spoiler [deg]

    % unit change
    londyn.B(:,1:3) = londyn.B(:,1:3)*180/pi; % Elevator, L/R Spoilers [deg] -> [rad]
    londyn.B(:,4:5) = londyn.B(:,4:5)*100; % L/R Throttles [%] -> []
    latdyn.B(:,1)   = latdyn.B(:,1)*100; % A Morphing [%] -> []
    latdyn.B(:,2:4) = latdyn.B(:,2:4)*180/pi; % Rudder, L/R Spoilers [deg] -> [rad]
    % tas [ft/s], alpha [rad], q [rad/s], theta [rad]
    % Elevator [rad], L Spoiler [rad], R Spoiler [rad]
    % L Throttle [], R Throttle []
    % beta [rad], p [rad/s], r [rad/s], phi [rad]
    % Aileron [rad], Rudder [rad], L Spoiler [rad], R Spoiler [rad]
    
    V0 = convvel(tas_trim(i),'kts','ft/s'); % trim speed [ft/s]
    alp_0 = deg2rad(Trimcond.alpha); % trim angle of attack [rad]
    gam_0 = deg2rad(Trimcond.pitch); % trim pitch angle [rad]
    
    % longitudinal dimensional derivatives
    X_V = londyn.A(1,1);
    X_alp = londyn.A(1,2);
    Z_V = londyn.A(2,1)*V0;
    Z_alp  = londyn.A(2,2)*V0;
    Z_q = (londyn.A(2,3)-1)*V0;
    M_V = londyn.A(3,1);
    M_alp = londyn.A(3,2);
    M_q = londyn.A(3,3);
    X_dele = londyn.B(1,2);
    Z_dele = londyn.B(2,1)*V0;
    M_dele = londyn.B(3,1);
    
    % longitudinal dynamics augmentation (altitude)
%     Alon = [londyn.A,zeros(4,1);
%             0,-V0,0,V0,0]; % V alp q theta h
%     Blon = [londyn.B(:,4)+londyn.B(:,5),londyn.B(:,1);
%             zeros(1,2)]; % delt dele
%     Clon = [eye(5);
%             Alon(1,:)
%             0,Z_alp,0,0,0]; % V alp q theta h Ax Az
%     Dlon = [zeros(6,2);
%             0,Z_dele];
%     londyn_aug = ss(Alon,Blon,Clon,Dlon);
%     set(londyn_aug,'Statename',{'tas','alpha','q','theta','alt'},...
%                    'Inputname',{'delt','dele'},...
%                    'Outputname',{'tas','alpha','q','theta','alt','Ax','Az'})

    % longitudinal dynamics augmentation (normal acceleration)
    Alon = [X_V X_alp/Z_alp 0 -g*cos(gam_0) X_dele-X_alp*Z_dele/Z_alp;
            0 Z_alp/V0 Z_alp 0 -Z_dele*w_ele;
            M_V M_alp/Z_alp M_q 0 M_dele-M_alp*Z_dele/Z_alp;
            0 0 1 0 0;
            0 0 0 0 -w_ele]; % V Az q theta dele
    Blon = [londyn.B(:,1) [0; Z_dele*w_ele; 0; 0];
            0 -w_ele]; % delt delec
    Clon = eye(5); % V Az q theta dele
    Dlon = zeros(5,2);
    londyn_aug = ss(Alon,Blon,Clon,Dlon);
    set(londyn_aug,'Statename',{'tas','Az','q','theta','dele'},...
                   'Inputname',{'delt','delec'},...
                   'Outputname',{'tas','Az','q','theta','dele'})

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

    disp('-------------------------------------------------------------------')
end
end
end

%% Longitudinal H-infinity control design (normal acceleration tracking)
omega_tas = 4;
zeta_tas = 1;
% omega_An = 4.6/2.4;
omega_An = 8;
zeta_An = 1;

% Weight function design
Model_tas = (omega_tas^2)/(s^2+2*zeta_tas*omega_tas*s+omega_tas^2);
% Model_An = omega_An/(s+omega_An);
Model_An = (omega_An^2)/(s^2+2*zeta_An*omega_An*s+omega_An^2);
Perf_tas = tf([0.1,5.973],[1,0.176]); 
Perf_An = tf([0.1,5.973],[1,0.176]);
D_thro = 0.1; 
D_elev = 0.1;
W_thro = tf([0.8,42.91],[1,119.2]); 
W_elev = tf([0.8,42.91],[1,119.2]);
N_tas = 0.01; % 0.06 knot
N_q = 0.06; % 0.3857 deg/s
N_the = 0.001; % 0.02 deg
N_Az = 0.01; % 0.0082 g
N_elev = 0.001; % 0.01 deg
N_Ax = 0.0016*g; % 0.0016 g

clearvars K_lon_hinfs gamma_lon_hinfs CL_lon_hinfs
for i = 1:num_tas
for j = 1:num_alt
for k = 1:num_mor
    LTI_lon = GTM_lon(:,:,i,j,k);
    
    % control interconnection
    systemnames        = 'LTI_lon Model_tas Model_An Perf_tas Perf_An D_thro D_elev W_thro W_elev N_tas N_Az N_q N_the N_elev';
    inputvar           = '[tas_c{1};An_c{1};Dist_thro{1};Dist_elev{1};Noise_tas{1};Noise_Az{1};Noise_q{1};Noise_the{1};Noise_elev{1};u{2}]';
    outputvar          = '[Perf_tas;Perf_An;W_thro;W_elev;tas_c-LTI_lon(1)-N_tas;An_c+LTI_lon(2)+N_Az;LTI_lon(3)+N_q;LTI_lon(4)+N_the;LTI_lon(5)+N_elev]';
    input_to_LTI_lon   = '[u(1)+D_thro;u(2)+D_elev]';
    input_to_Model_tas = '[tas_c]';
    input_to_Model_An  = '[An_c]';
    input_to_Perf_tas  = '[Model_tas-LTI_lon(1)]';
    input_to_Perf_An   = '[Model_An+LTI_lon(2)]';
    input_to_D_thro    = '[Dist_thro]';
    input_to_D_elev    = '[Dist_elev]';
    input_to_W_thro    = '[u(1)+D_thro]';
    input_to_W_elev    = '[u(2)+D_elev]';
    input_to_N_tas     = '[Noise_tas]';
    input_to_N_Az      = '[Noise_Az]';
    input_to_N_q       = '[Noise_q]';
    input_to_N_the     = '[Noise_the]';
    input_to_N_elev    = '[Noise_elev]';
    sysoutname         = 'LTI_G_lon';
    cleanupsysic       = 'yes'; 
    sysic
    
    % Synthesis
    [K_lon_hinf,~,gamma_lon_hinf] = hinfsyn(LTI_G_lon,5,2);
    gamma_lon_hinf
    gamma_lon_hinfs(i,j,k) = gamma_lon_hinf;
    K_lon_hinfs(:,:,i,j,k) = K_lon_hinf;
    
    % Closed-loop interconnection
    systemnames         = 'LTI_lon K_lon_hinf';
    inputvar            = '[tas_c{1};An_c{1}]';
    outputvar           = '[LTI_lon]';
    input_to_LTI_lon    = '[K_lon_hinf]';
    input_to_K_lon_hinf = '[tas_c-LTI_lon(1);An_c+LTI_lon(2);LTI_lon(3);LTI_lon(4);LTI_lon(5)]';
    sysoutname          = 'CL_lon_hinf';
    cleanupsysic        = 'yes'; 
    sysic

    CL_lon_hinfs(:,:,i,j,k) = CL_lon_hinf;
end
end
end

step(CL_lon_hinf,10)
grid on

%% Longitudinal H-infinity control design (altitude tracking)
% Weight function design
% Model_tas = tf(16,[1,5.6,16])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
% Model_alt = tf(16,[1,5.6,16])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
% Perf_tas = tf([0.1,5.973],[1,0.176]); Perf_alt = tf([0.1,5.973],[1,0.176]);
% D_thro = 0.01; D_elev = 0.01;
% W_thro = tf([0.8,42.91],[1,119.2]); W_elev = tf([0.8,42.91],[1,119.2]);
% N_tas = 0.01; N_alp = 0.01; N_q = 0.1; N_the = 0.01;
% N_alt = 0.01; N_Ax = 0.01; N_Az = 0.01;
% 
% clearvars K_lon_hinfs gamma_lon_hinfs CL_lon_hinfs
% for i = 1:num_tas
% for j = 1:num_alt
% for k = 1:num_mor
%     LTI_lon = GTM_lon(:,:,i,j,k);
%     
%     % control interconnection
%     systemnames        = 'LTI_lon Model_tas Model_alt Perf_tas Perf_alt D_thro D_elev W_thro W_elev N_tas N_alp N_q N_the N_alt N_Ax N_Az';
%     inputvar           = '[tas_c{1};alt_c{1};Dist_thro{1};Dist_elev{1};Noise_tas{1};Noise_alp{1};Noise_q{1};Noise_the{1};Noise_alt{1};Noise_Ax{1};Noise_Az{1};u{2}]';
%     outputvar          = '[Perf_tas;Perf_alt;W_thro;W_elev;tas_c-LTI_lon(1)-N_tas;alt_c-LTI_lon(5)-N_alt;LTI_lon(2)+N_alp;LTI_lon(3)+N_q;LTI_lon(4)+N_the;LTI_lon(6)+N_Ax;LTI_lon(7)+N_Az]';
%     input_to_LTI_lon   = '[u(1)+D_thro;u(2)+D_elev]';
%     input_to_Model_tas = '[tas_c]';
%     input_to_Model_alt = '[alt_c]';
%     input_to_Perf_tas  = '[Model_tas-LTI_lon(1)]';
%     input_to_Perf_alt  = '[Model_alt-LTI_lon(5)]';
%     input_to_D_thro    = '[Dist_thro]';
%     input_to_D_elev    = '[Dist_elev]';
%     input_to_W_thro    = '[u(1)+D_thro]';
%     input_to_W_elev    = '[u(2)+D_elev]';
%     input_to_N_tas     = '[Noise_tas]';
%     input_to_N_alp     = '[Noise_alp]';
%     input_to_N_q       = '[Noise_q]';
%     input_to_N_the     = '[Noise_the]';
%     input_to_N_alt     = '[Noise_alt]';
%     input_to_N_Ax      = '[Noise_Ax]';
%     input_to_N_Az      = '[Noise_Az]';
%     sysoutname         = 'LTI_G_lon';
%     cleanupsysic       = 'yes'; 
%     sysic
%     
%     % Synthesis
%     [K_lon_hinf,~,gamma_lon_hinf] = hinfsyn(LTI_G_lon,7,2);
%     gamma_lon_hinf
%     gamma_lon_hinfs(i,j,k) = gamma_lon_hinf;
%     K_lon_hinfs(:,:,i,j,k) = K_lon_hinf;
%     
%     % Closed-loop interconnection
%     systemnames         = 'LTI_lon K_lon_hinf';
%     inputvar            = '[tas_c{1};alt_c{1}]';
%     outputvar           = '[LTI_lon]';
%     input_to_LTI_lon    = '[K_lon_hinf]';
%     input_to_K_lon_hinf = '[tas_c-LTI_lon(1);alt_c-LTI_lon(5);LTI_lon(2);LTI_lon(3);LTI_lon(4);LTI_lon(6);LTI_lon(7)]';
%     sysoutname          = 'CL_lon_hinf';
%     cleanupsysic        = 'yes'; 
%     sysic
% 
%     CL_lon_hinfs(:,:,i,j,k) = CL_lon_hinf;
% end
% end
% end
% 
% step(CL_lon_hinf,10)
% grid on

% Frequency domain analysis
% w = logspace(-2,2,50);
% for i = 1:num_tas
% for j = 1:num_alt
% for k = 1:num_mor
%     trim_point = (i-1)*num_alt*num_mor + (j-1)*num_mor + k;
%     [mag_P_lon,phase_P_lon] = bode(GTM_lon(:,:,i,j,k),w);
%     mag_elev_to_alpha_P_lon(:,trim_point) = mag2db(squeeze(mag_P_lon(1,1,:)));
%     phase_elev_to_alpha_P_lon(:,trim_point) = squeeze(phase_P_lon(1,1,:));
%     [mag_CL_lon_hinfs,phase_CL_lon_hinfs] = bode(CL_lon_hinfs(:,:,i,j,k),w);
%     mag_elev_to_alpha_CL_lon_hinfs(:,trim_point) = mag2db(squeeze(mag_CL_lon_hinfs(1,1,:))).';
%     phase_elev_to_alpha_CL_lon_hinfs(:,trim_point) = squeeze(phase_CL_lon_hinfs(1,1,:)).';
% end
% end
% end
% 
% [mag_Model_tas_raw,phase_Model_tas_raw] = bode(Model_tas,w);
% mag_Model_tas = mag2db(squeeze(mag_Model_tas_raw(1,1,:)));
% phase_Model_tas = squeeze(phase_Model_tas_raw(1,1,:));
% 
% [mag_Model_alp_raw,phase_Model_alp_raw] = bode(Model_alp,w);
% mag_Model_alp = mag2db(squeeze(mag_Model_alp_raw(1,1,:)));
% phase_Model_alp = squeeze(phase_Model_alp_raw(1,1,:));
% 
% figure
% subplot(2,1,1)
% semilogx(w,mag_elev_to_alpha_P_lon,'-b','linewidth',2)
% hold on
% semilogx(w,mag_Model_alp,'-r','linewidth',2)
% semilogx(w,mag_elev_to_alpha_CL_lon_hinfs,'--k','linewidth',2)
% legend('Open Loop','Tracking Model','Closed Loop')
% grid on
% ylabel('Magnitude, dB')
% subplot(2,1,2)
% semilogx(w,phase_elev_to_alpha_P_lon,'-b','linewidth',2)
% legend('Open Loop')
% hold on
% semilogx(w,phase_Model_alp-360,'-r','linewidth',2)
% semilogx(w,phase_elev_to_alpha_CL_lon_hinfs,'--k','linewidth',2)
% legend('Open Loop','Tracking Model','Closed Loop')
% grid on
% ylabel('Phase, deg')
% xlabel('Frequency, rad/s')

%% Lateral H-infinity control design
omega_beta = 4.6/0.707/1.5;
zeta_beta = 0.707;
omega_phi = 4.6/0.707/1.5;
zeta_phi = 0.707;

% Weight function
% Model_bet = tf(1.25^2,[1 2.5 1.25^2]);
Model_bet = tf(omega_beta^2,[1 2*zeta_beta*omega_beta omega_beta^2]);
% Model_phi = tf(2^2,[1 4 2^2]);
Model_phi = tf(omega_phi^2,[1 2*zeta_phi*omega_phi omega_phi^2]);
Perf_bet = tf([0.1,5.973],[1,0.176]);
Perf_phi = tf([0.1,5.973],[1,0.176]);
D_ail = 0.01;
D_rud = 0.01;
W_ail = tf([0.8,42.91],[1,119.2]);
W_rud = tf([0.8,42.91],[1,119.2]);
N_bet = 0.01; % 0.0330 deg
N_p = 0.1; % 0.5220 deg/s
N_r = 0.1; % 0.3330 deg/s
N_phi = 0.01; % 0.02 deg
N_Ay = 0.1; % 0.0082 g

clearvars K_lat_hinfs gamma_lat_hinfs CL_lat_hinfs
for i = 1:num_tas
for j = 1:num_alt
for k = 1:num_mor
    LTI_lat = GTM_lat(:,:,i,j,k);
    
    % control interconnection
    systemnames        = 'LTI_lat Model_bet Model_phi Perf_bet Perf_phi D_ail D_rud W_ail W_rud N_bet N_p N_r N_phi N_Ay';
    inputvar           = '[bet_c{1};phi_c{1};Dist_ail{1};Dist_rud{1};Noise_bet{1};Noise_p{1};Noise_r{1};Noise_phi{1};Noise_Ay{1};u{2}]';
    outputvar          = '[Perf_bet;Perf_phi;W_ail;W_rud;bet_c-LTI_lat(1)-N_bet;phi_c-LTI_lat(4)-N_phi;LTI_lat(2)+N_p;LTI_lat(3)+N_r;LTI_lat(5)+N_Ay]';
    input_to_LTI_lat   = '[u(1)+D_ail;u(2)+D_rud]';
    input_to_Model_bet = '[bet_c]';
    input_to_Model_phi = '[phi_c]';
    input_to_Perf_bet  = '[Model_bet-LTI_lat(1)]';
    input_to_Perf_phi  = '[Model_phi-LTI_lat(4)]';
    input_to_D_ail     = '[Dist_ail]';
    input_to_D_rud     = '[Dist_rud]';
    input_to_W_ail     = '[u(1)+D_ail]';
    input_to_W_rud     = '[u(2)+D_rud]';
    input_to_N_bet     = '[Noise_bet]';
    input_to_N_p       = '[Noise_p]';
    input_to_N_r       = '[Noise_r]';
    input_to_N_phi     = '[Noise_phi]';
    input_to_N_Ay      = '[Noise_Ay]';
    sysoutname         = 'LTI_G_lat';
    cleanupsysic       = 'yes'; 
    sysic

    % Controller synthesis
    [K_lat_hinf,~,gamma_lat_hinf] = hinfsyn(LTI_G_lat,5,2);
    gamma_lat_hinf
    gamma_lat_hinfs(i,j,k) = gamma_lat_hinf;
    K_lat_hinfs(:,:,i,j,k) = K_lat_hinf;
    
    % Closed-loop interconnection
    systemnames         = 'LTI_lat K_lat_hinf';
    inputvar            = '[bet_c{1};phi_c{1}]';
    outputvar           = '[LTI_lat]';
    input_to_LTI_lat    = '[K_lat_hinf]';
    input_to_K_lat_hinf = '[bet_c-LTI_lat(1);phi_c-LTI_lat(4);LTI_lat(2);LTI_lat(3);LTI_lat(5)]';
    sysoutname          = 'CL_lat_hinf';
    cleanupsysic        = 'yes'; 
    sysic

    CL_lat_hinfs(:,:,i,j,k) = CL_lat_hinf;
end
end
end

% bode(LTI_lat,logspace(-2,2,50))
step(CL_lat_hinf,10), grid on

% [y,t] = step(CL_lat_hinf,10);
% [yb,tb] = step(Model_bet,10);
% [yp,tp] = step(Model_phi,10);
% figure
% subplot(1,2,1), plot(tb,yb)
% hold on, plot(t,y(:,1,1)), hold off
% grid on
% legend('Model_\beta','Closed-loop')
% subplot(1,2,2), plot(tp,yp)
% hold on, plot(t,y(:,4,2)), hold off
% grid on
% legend('Model_\phi','Closed-loop')

%% LPV modeling
Domain = rgrid(tas_param,alt_param,mor_param);
P_lon = pss(GTM_lon,Domain);
P_lat = pss(GTM_lat,Domain);

%% Longitudinal LPV control design
% Basis function
b1 = basis(1,0);
btas = basis(tas_param,'tas_param',1);
balt = basis(alt_param,'alt_param',1);
bmor = basis(mor_param,'mor_param',1);
Xb = [b1;btas;balt;bmor;btas^2;balt^2;bmor^2];
Yb = Xb;

clearvars K_lon gamma_lon CL_lon
for i = 1:num_tas
for j = 1:num_alt
for k = 1:num_mor        
    % control interconnection
    systemnames        = 'P_lon Model_tas Model_alt Perf_tas Perf_alt D_thro D_elev W_thro W_elev N_tas N_alp N_q N_the N_alt N_Ax N_Az';
    inputvar           = '[tas_c{1};alt_c{1};Dist_thro{1};Dist_elev{1};Noise_tas{1};Noise_alp{1};Noise_q{1};Noise_the{1};Noise_alt{1};Noise_Ax{1};Noise_Az{1};u{2}]';
    outputvar          = '[Perf_tas;Perf_alt;W_thro;W_elev;tas_c-P_lon(1)-N_tas;alt_c-P_lon(5)-N_alt;P_lon(2)+N_alp;P_lon(3)+N_q;P_lon(4)+N_the;P_lon(6)+N_Ax;P_lon(7)+N_Az]';
    input_to_P_lon     = '[u(1)+D_thro;u(2)+D_elev]';
    input_to_Model_tas = '[tas_c]';
    input_to_Model_alt = '[alt_c]';
    input_to_Perf_tas  = '[Model_tas-P_lon(1)]';
    input_to_Perf_alt  = '[Model_alt-P_lon(5)]';
    input_to_D_thro    = '[Dist_thro]';
    input_to_D_elev    = '[Dist_elev]';
    input_to_W_thro    = '[u(1)+D_thro]';
    input_to_W_elev    = '[u(2)+D_elev]';
    input_to_N_tas     = '[Noise_tas]';
    input_to_N_alp     = '[Noise_alp]';
    input_to_N_q       = '[Noise_q]';
    input_to_N_the     = '[Noise_the]';
    input_to_N_alt     = '[Noise_alt]';
    input_to_N_Ax      = '[Noise_Ax]';
    input_to_N_Az      = '[Noise_Az]';
    sysoutname         = 'G_lon';
    cleanupsysic       = 'yes'; 
    sysic
    
    % synthesis
    if RateBoundOn == 1
        [K_lon,gamma_lon,info_lon] = lpvsyn(G_lon,7,2,Xb,Yb);
    else
        [K_lon,gamma_lon,info_lon] = lpvsyn(G_lon,7,2);
    end
    
    % Closed-loop interconnection
    systemnames    = 'P_lon K_lon';
    inputvar       = '[tas_c{1};alt_c{1}]';
    outputvar      = '[P_lon]';
    input_to_P_lon = '[K_lon]';
    input_to_K_lon = '[tas_c-P_lon(1);alt_c-P_lon(5);P_lon(2);P_lon(3);P_lon(4);P_lon(6);P_lon(7)]';
    sysoutname     = 'CL_lon';
    cleanupsysic   = 'yes'; 
    sysic
end
end
end

%% Lateral LPV control design
clearvars K_lat_hinfs gamma_lat_hinfs CL_lat_hinfs
for i = 1:num_tas
for j = 1:num_alt
for k = 1:num_mor
    % control interconnection
    systemnames        = 'P_lat Model_bet Model_phi Perf_bet Perf_phi D_ail D_rud W_ail W_rud N_bet N_p N_r N_phi N_Ay';
    inputvar           = '[bet_c{1};phi_c{1};Dist_ail{1};Dist_rud{1};Noise_bet{1};Noise_p{1};Noise_r{1};Noise_phi{1};Noise_Ay{1};u{2}]';
    outputvar          = '[Perf_bet;Perf_phi;W_ail;W_rud;bet_c-P_lat(1)-N_bet;phi_c-P_lat(4)-N_phi;P_lat(2)+N_p;P_lat(3)+N_r;P_lat(5)+N_Ay]';
    input_to_P_lat     = '[u(1)+D_ail;u(2)+D_rud]';
    input_to_Model_bet = '[bet_c]';
    input_to_Model_phi = '[phi_c]';
    input_to_Perf_bet  = '[Model_bet-P_lat(1)]';
    input_to_Perf_phi  = '[Model_phi-P_lat(4)]';
    input_to_D_ail     = '[Dist_ail]';
    input_to_D_rud     = '[Dist_rud]';
    input_to_W_ail     = '[u(1)+D_ail]';
    input_to_W_rud     = '[u(2)+D_rud]';
    input_to_N_bet     = '[Noise_bet]';
    input_to_N_p       = '[Noise_p]';
    input_to_N_r       = '[Noise_r]';
    input_to_N_phi     = '[Noise_phi]';
    input_to_N_Ay      = '[Noise_Ay]';
    sysoutname         = 'G_lat';
    cleanupsysic       = 'yes'; 
    sysic

    % synthesis
    if RateBoundOn == 1
        [K_lat,gamma_lat,info_lat] = lpvsyn(G_lat,5,2,Xb,Yb);
    else
        [K_lat,gamma_lat,info_lat] = lpvsyn(G_lat,5,2);
    end
    
    % Closed-loop interconnection
    systemnames    = 'P_lat K_lat';
    inputvar       = '[bet_c{1};phi_c{1}]';
    outputvar      = '[P_lat]';
    input_to_P_lat = '[K_lat]';
    input_to_K_lat = '[bet_c-P_lat(1);phi_c-P_lat(4);P_lat(2);P_lat(3);P_lat(5)]';
    sysoutname     = 'CL_lat';
    cleanupsysic   = 'yes'; 
    sysic
end
end
end

%% Nonlinear Simulation
vehicle='GTM_T2';
noise_on = 0;
fuelburn_on = 0;

MWS.StatesInp
target            = struct('gamma',0,'tas',80); % target trim condition [knot]
MWS.StatesInp(9)  = 800; % set altitude [ft]
MWS.bias.morphing = -12.5; % set morphing parameter

loadmws(MWS,'gtm_design'); % load MWS

[MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose); % trim calculation

[tsim,xsim,ysim]=sim('gtm_design',[0 15]);

% Grab state in ysim First 6 are trim outputs, next 12 are state.
% ysim=ysim(:,7:18);

%% Plot results
% set(figure(1),'Position',[20 80 900 700]);
% subplot(321),
% plot(tsim,ysim(:,1)); grid on
% title('Linear Velocity to Doublet Sequence [elevator,rudder,aileron]');
% xlabel('Time (sec)');ylabel('u (ft/sec)')
% legend('Simulation', 'Linear');
% subplot(323),
% plot(tsim,ysim(:,2)); grid on
% xlabel('Time (sec)');ylabel('v (ft/sec)')
% subplot(325),
% plot(tsim,ysim(:,3)); grid on
% xlabel('Time (sec)');ylabel('w (ft/sec)')
% 
% subplot(322),
% plot(tsim,180/pi*ysim(:,4)); grid on
% title('Angular Velocity to Doublet Sequence [elevator,rudder,aileron]');
% xlabel('Time (sec)');ylabel('p (deg/sec)')
% subplot(324),
% plot(tsim,180/pi*ysim(:,5)); grid on
% xlabel('Time (sec)');ylabel('q (deg/sec)')
% subplot(326),
% plot(tsim,180/pi*ysim(:,6)); grid on
% xlabel('Time (sec)');ylabel('r (deg/sec)')
