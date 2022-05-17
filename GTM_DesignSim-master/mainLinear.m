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
tas_grid  = [80]; % 70~85 [kts]
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

%% Trimming
for i = 1:num_tas
for j = 1:num_alt
for k = 1:num_mor
    trim_point = (i-1)*num_alt*num_mor + (j-1)*num_mor + k; % trim point number

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
    throttle_trim(i,j,k) = Trimcond.throttle;
    aileron_trim(i,j,k)  = Trimcond.aileron;
    elevator_trim(i,j,k) = Trimcond.elevator;
    rudder_trim(i,j,k)   = Trimcond.rudder;

    if (Err < 1e-3)
        disp(['TRIM @ ',num2str(trim_point),'/',num2str(num_tas*num_alt*num_mor),' : DONE'])
    else
        error('TRIM FAIL')
    end
    disp('-------------------------------------------------------------------')
end
end
end

%% Linearization
f_ele = 5; % elevator frequency [Hz], W_ele = f_ele/(s+f_ele)
f_ail = 5; % morphing actuator frequency [Hz], W_ele = f_ail/(s+f_ail)
f_rud = 5; % rudder frequency [Hz], W_ele = f_rud/(s+f_rud)

clearvars GTM_lon GTM_lat
for i = 1:num_tas
for j = 1:num_alt
for k = 1:num_mor
    trim_point = (i-1)*num_alt*num_mor + (j-1)*num_mor + k;
    
    % linearization
    [gtmdyn,londyn,latdyn]=linmodel(MWS_grid(i,j,k),vabflag,use_all_inputs,Ts);

    % unit change
    londyn.B(:,1:3) = londyn.B(:,1:3)*180/pi; % [deg] -> [rad]
    londyn.B(:,4:5) = londyn.B(:,4:5)*100; % [%] -> []
    latdyn.B(:,1)   = latdyn.B(:,1)*100; % [%] -> []
    latdyn.B(:,2:4) = latdyn.B(:,2:4)*180/pi; % [deg] -> [rad]

    V0 = tas_grid(i); % trim speed
    
    % longitudinal dimensional derivatives
    Z_dele = londyn.B(2,1)*V0;
    Z_alp  = londyn.A(2,2)*V0;
    M_dele = londyn.B(3,1);
    M_alp  = londyn.A(3,2);
    
    % longitudinal dynamics augmentation (short-period mode)
%     Alon = londyn.A(2:3,2:3);
%     Blon = londyn.B(2:3,1);
%     Clon = eye(2);
%     Dlon = zeros(2,1);
%     londyn_aug = ss(Alon,Blon,Clon,Dlon);
%     set(londyn_aug,'Statename',{'alpha','q'},...
%                    'Inputname',{'dele'},...
%                    'Outputname',{'alpha','q'})

    % longitudinal dynamics augmentation (full dynamics)
    Alon = [londyn.A,zeros(4,1);
            0,-V0,0,V0,0]; % V alp q theta h
    Blon = [londyn.B(:,4)+londyn.B(:,5),londyn.B(:,1);
            zeros(1,2)]; % delt delec
    Clon = [eye(5);
            Alon(1,:)
            0,Z_alp,0,0,0]; % V alp q theta h Ax Az
    Dlon = [zeros(6,2);
            0,Z_dele];
    londyn_aug = ss(Alon,Blon,Clon,Dlon);
    set(londyn_aug,'Statename',{'tas','alpha','q','theta','alt'},...
                   'Inputname',{'delt','dele'},...
                   'Outputname',{'tas','alpha','q','theta','alt','Ax','Az'})

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

%% Longitudinal H-infinity control design (short-period mode)
% Weight function design
% Model_alp = tf(16,[1,5.6,16])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
% Perf_alp = tf([0.1,5.973],[1,0.176]);
% D_elev = 0.01;
% W_elev = tf([0.8,42.91],[1,119.2]);
% N_alp = 0.01;
% N_q = 0.1;
% 
% clearvars K_lon_hinfs gamma_lon_hinfs CL_lon_hinfs
% for i = 1:num_tas
% for j = 1:num_alt
% for k = 1:num_mor
%     trim_point = (i-1)*num_alt*num_mor + (j-1)*num_mor + k;
%     
%     LTI_lon = GTM_lon(:,:,i,j,k);
%     
%     % LTI control interconnection
%     systemnames        = 'LTI_lon Model_alp Perf_alp D_elev W_elev N_alp N_q';
%     inputvar           = '[alp_c{1};Dist_elev{1};Noise_alp{1};Noise_q{1};u{1}]';
%     outputvar          = '[Perf_alp;W_elev;alp_c-LTI_lon(1)-N_alp;LTI_lon(2)+N_q]';
%     input_to_LTI_lon   = '[u+D_elev]';
%     input_to_Model_alp = '[alp_c]';
%     input_to_Perf_alp  = '[Model_alp-LTI_lon(1)]';
%     input_to_D_elev    = '[Dist_elev]';
%     input_to_W_elev    = '[u+D_elev]';
%     input_to_N_alp     = '[Noise_alp]';
%     input_to_N_q       = '[Noise_q]';
%     sysoutname         = 'LTI_G_lon';
%     cleanupsysic       = 'yes'; 
%     sysic
%     
%     % Synthesis
%     [K_lon_hinf,~,gamma_lon_hinf] = hinfsyn(LTI_G_lon,2,1);
%     gamma_lon_hinf
%     gamma_lon_hinfs(i,j,k) = gamma_lon_hinf;
%     K_lon_hinfs(:,:,i,j,k) = K_lon_hinf;
%     
%     % Closed-loop interconnection
%     systemnames         = 'LTI_lon K_lon_hinf';
%     inputvar            = '[alp_c{1}]';
%     outputvar           = '[LTI_lon]';
%     input_to_LTI_lon    = '[K_lon_hinf]';
%     input_to_K_lon_hinf = '[alp_c-LTI_lon(1);LTI_lon(2)]';
%     sysoutname          = 'CL_lon_hinf';
%     cleanupsysic        = 'yes'; 
%     sysic
% 
%     CL_lon_hinfs(:,:,i,j,k) = CL_lon_hinf;
% end
% end
% end
% 
% % Frequency domain analysis
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

%% Longitudinal H-infinity control design (full dynamics)
% Weight function design
Model_tas = tf(16,[1,5.6,16])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
Model_alt = tf(16,[1,5.6,16])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
Perf_tas = tf([0.1,5.973],[1,0.176]);
Perf_alt = tf([0.1,5.973],[1,0.176]);
D_thro = 0.01;
D_elev = 0.01;
W_thro = tf([0.8,42.91],[1,119.2]);
W_elev = tf([0.8,42.91],[1,119.2]);
N_tas = 0.01;
N_alp = 0.01;
N_q = 0.1;
N_the = 0.01;
N_alt = 0.01;
N_Ax = 0.01;
N_Az = 0.01;

clearvars K_lon_hinfs gamma_lon_hinfs CL_lon_hinfs
for i = 1:num_tas
for j = 1:num_alt
for k = 1:num_mor
    trim_point = (i-1)*num_alt*num_mor + (j-1)*num_mor + k;
    
    LTI_lon = GTM_lon(:,:,i,j,k);
    
    % LTI control interconnection
    systemnames        = 'LTI_lon Model_tas Model_alt Perf_tas Perf_alt D_thro D_elev W_thro W_elev N_tas N_alp N_q N_the N_alt N_Ax N_Az';
    inputvar           = '[tas_c{1};alt_c{1};Dist_thro{1};Dist_elev{1};Noise_tas{1};Noise_alp{1};Noise_q{1};Noise_the{1};Noise_alt{1};Noise_Ax{1};Noise_Az{1};u{2}]';
    outputvar          = '[Perf_tas;Perf_alt;W_thro;W_elev;tas_c-LTI_lon(1)-N_tas;alt_c-LTI_lon(5)-N_alt;LTI_lon(2)+N_alp;LTI_lon(3)+N_q;LTI_lon(4)+N_the;LTI_lon(6)+N_Ax;LTI_lon(7)+N_Az]';
    input_to_LTI_lon   = '[u(1)+D_thro;u(2)+D_elev]';
    input_to_Model_tas = '[tas_c]';
    input_to_Model_alt = '[alt_c]';
    input_to_Perf_tas  = '[Model_tas-LTI_lon(1)]';
    input_to_Perf_alt  = '[Model_alt-LTI_lon(5)]';
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
    sysoutname         = 'LTI_G_lon';
    cleanupsysic       = 'yes'; 
    sysic
    
    % Synthesis
    [K_lon_hinf,~,gamma_lon_hinf] = hinfsyn(LTI_G_lon,7,2);
    gamma_lon_hinf
    gamma_lon_hinfs(i,j,k) = gamma_lon_hinf;
    K_lon_hinfs(:,:,i,j,k) = K_lon_hinf;
    
    % Closed-loop interconnection
    systemnames         = 'LTI_lon K_lon_hinf';
    inputvar            = '[tas_c{1};alt_c{1}]';
    outputvar           = '[LTI_lon]';
    input_to_LTI_lon    = '[K_lon_hinf]';
    input_to_K_lon_hinf = '[tas_c-LTI_lon(1);alt_c-LTI_lon(5);LTI_lon(2);LTI_lon(3);LTI_lon(4);LTI_lon(6);LTI_lon(7)]';
    sysoutname          = 'CL_lon_hinf';
    cleanupsysic        = 'yes'; 
    sysic

    CL_lon_hinfs(:,:,i,j,k) = CL_lon_hinf;
end
end
end

% Frequency domain analysis
w = logspace(-2,2,50);
for i = 1:num_tas
for j = 1:num_alt
for k = 1:num_mor
    trim_point = (i-1)*num_alt*num_mor + (j-1)*num_mor + k;
    [mag_P_lon,phase_P_lon] = bode(GTM_lon(:,:,i,j,k),w);
    mag_elev_to_alpha_P_lon(:,trim_point) = mag2db(squeeze(mag_P_lon(1,1,:)));
    phase_elev_to_alpha_P_lon(:,trim_point) = squeeze(phase_P_lon(1,1,:));
    [mag_CL_lon_hinfs,phase_CL_lon_hinfs] = bode(CL_lon_hinfs(:,:,i,j,k),w);
    mag_elev_to_alpha_CL_lon_hinfs(:,trim_point) = mag2db(squeeze(mag_CL_lon_hinfs(1,1,:))).';
    phase_elev_to_alpha_CL_lon_hinfs(:,trim_point) = squeeze(phase_CL_lon_hinfs(1,1,:)).';
end
end
end

[mag_Model_alp_raw,phase_Model_alp_raw] = bode(Model_alp,w);
mag_Model_alp = mag2db(squeeze(mag_Model_alp_raw(1,1,:)));
phase_Model_alp = squeeze(phase_Model_alp_raw(1,1,:));

figure
subplot(2,1,1)
semilogx(w,mag_elev_to_alpha_P_lon,'-b','linewidth',2)
hold on
semilogx(w,mag_Model_alp,'-r','linewidth',2)
semilogx(w,mag_elev_to_alpha_CL_lon_hinfs,'--k','linewidth',2)
legend('Open Loop','Tracking Model','Closed Loop')
grid on
ylabel('Magnitude, dB')
subplot(2,1,2)
semilogx(w,phase_elev_to_alpha_P_lon,'-b','linewidth',2)
legend('Open Loop')
hold on
semilogx(w,phase_Model_alp-360,'-r','linewidth',2)
semilogx(w,phase_elev_to_alpha_CL_lon_hinfs,'--k','linewidth',2)
legend('Open Loop','Tracking Model','Closed Loop')
grid on
ylabel('Phase, deg')
xlabel('Frequency, rad/s')

%% Lateral H-infinity control design
% Weight function
Model_bet = tf(1.25^2,[1 2.5 1.25^2]);
Model_phi = tf(2^2,[1 4 2^2]);
Perf_bet = tf([0.1,5.973],[1,0.176]);
Perf_phi = tf([0.1,5.973],[1,0.176]);
D_ail = 0.01;
D_rud = 0.01;
W_ail = tf([0.8,42.91],[1,119.2]);
W_rud = tf([0.8,42.91],[1,119.2]);
N_bet = 0.01;
N_p = 0.1;
N_r = 0.1;
N_phi = 0.01;
N_Ay = 0.1;

clearvars K_lat_hinfs gamma_lat_hinfs CL_lat_hinfs
for i = 1:num_tas
for j = 1:num_alt
for k = 1:num_mor
    LTI_lat = GTM_lat(:,:,i,j,k);
    
    % Control design interconnection
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
% step(CL_lat_hinf)

[y,t] = step(CL_lat_hinf,10);
[yb,tb] = step(Model_bet,10);
[yp,tp] = step(Model_phi,10);
% figure
subplot(1,2,1), plot(tb,yb)
hold on, plot(t,y(:,1,1)), hold off
grid on
legend('Model_\beta','Closed-loop')
subplot(1,2,2), plot(tp,yp)
hold on, plot(t,y(:,4,2)), hold off
grid on
legend('Model_\phi','Closed-loop')

%% LPV modeling
Domain = rgrid(tas_param,alt_param,mor_param);
P_lon = pss(GTM_lon,Domain);
P_lat = pss(GTM_lat,Domain);

%% Longitudinal LPV control design
% Control interconnection
systemnames = ' P_lon Model_alp Perf_alp D_elev W_elev N_alp N_q ';
inputvar = '[ Pilot_lon{1}; Dist_elev{1}; Noise_alp{1}; Noise_q{1}; u ]';
outputvar = '[ Perf_alp; W_elev; Pilot_lon-P_lon(1)-N_alp; P_lon(2)+N_q ]';
input_to_P_lon = '[ u+D_elev ]';
input_to_Model_alp = '[ Pilot_lon ]';
input_to_Perf_alp = '[ Model_alp-P_lon(1) ]';
input_to_D_elev = '[ Dist_elev ]';
input_to_W_elev = '[ u+D_elev ]';
input_to_N_alp = '[ Noise_alp ]';
input_to_N_q = '[ Noise_q ]';
sysoutname = 'G_lon';
cleanupsysic = 'yes'; 
sysic

% Basis function
b1 = basis(1,0);
btas = basis(tas_param,'tas_param',1);
balt = basis(alt_param,'alt_param',1);
bmor = basis(mor_param,'mor_param',1);
Xb = [b1;btas;balt;bmor;btas^2;balt^2;bmor^2];
Yb = Xb;

% Synthesis
tic
if RateBoundOn == 1
    [K_lon,gamma_lon,info_lon] = lpvsyn(G_lon,nmeas_lon,ncont_lon,Xb,Yb);
else
    [K_lon,gamma_lon,info_lon] = lpvsyn(G_lon,nmeas_lon,ncont_lon);
end
disp('LPV SYNTHESIS DONE')
gamma_lon
toc

% Closed-loop interconnection
systemnames = ' P_lon Model_alp Perf_alp W_elev D_elev N_alp N_q K_lon ';
inputvar = '[ Pilot_lon{1}; Dist_elev{1}; Noise_alp{1}; Noise_q{1} ]';
outputvar = '[ P_lon ]';
input_to_P_lon = '[ K_lon+D_elev ]';
input_to_Model_alp = '[ Pilot_lon ]';
input_to_Perf_alp = '[ Model_alp-P_lon(1) ]';
input_to_W_elev = '[ K_lon+D_elev ]';
input_to_D_elev = '[ Dist_elev ]';
input_to_N_alp = '[ Noise_alp ]';
input_to_N_q = '[ Noise_q ]';
input_to_K_lon = '[ Pilot_lon-P_lon(1)-N_alp; P_lon(2)+N_q ]';
sysoutname = 'CL_lon';
cleanupsysic = 'yes'; 
sysic


%% Lateral LPV control design
% Control interconnection
systemnames = ' P_lat Model_bet Model_p Perf_bet Perf_p D_ail D_rud W_ail W_rud N_bet N_p N_r ';
inputvar = '[ Pilot_lat{1}; Pilot_ped{1}; Dist_ail{1}; Dist_rud{1}; Noise_bet{1}; Noise_p{1}; Noise_r{1}; u{2} ]';
outputvar = '[ Perf_bet; Perf_p; W_ail; W_rud; Pilot_lat; Pilot_ped-P_lat(1)-N_bet; P_lat(2)+N_p; P_lat(3)+N_r ]';
input_to_P_lat = '[ u(1)+D_ail; u(2)+D_rud ]';
input_to_Model_bet = '[ Pilot_ped ]';
input_to_Model_p = '[ Pilot_lat ]';
input_to_Perf_bet = '[ Model_bet-P_lat(1) ]';
input_to_Perf_p = '[ Model_p-P_lat(2) ]';
input_to_D_ail = '[ Dist_ail ]';
input_to_D_rud = '[ Dist_rud ]';
input_to_W_ail = '[ u(1)+D_ail ]';
input_to_W_rud = '[ u(2)+D_rud ]';
input_to_N_bet = '[ Noise_bet ]';
input_to_N_p = '[ Noise_p ]';
input_to_N_r = '[ Noise_r ]';
sysoutname = 'G_lat';
cleanupsysic = 'yes'; 
sysic

% Synthesis
% tic
% if rateBound == 1
%     [K_lat,gamma_lat,info_lat] = lpvsyn(G_lat,nmeas_lat,ncont_lat,Xb,Yb);
% else
%     [K_lat,gamma_lat,info_lat] = lpvsyn(G_lat,nmeas_lat,ncont_lat);
% end
% disp('LPV SYNTHESIS DONE')
% gamma_lat
% toc

% Closed-loop interconnection
% systemnames = ' P_lat Model_bet Model_p Perf_bet Perf_p D_ail D_rud W_ail W_rud N_bet N_p N_r K_lat ';
% inputvar = '[ Pilot_lat{1}; Pilot_ped{1}; Dist_ail{1}; Dist_rud{1}; Noise_bet{1}; Noise_p{1}; Noise_r{1} ]';
% outputvar = '[ Perf_bet; Perf_p; W_ail; W_rud ]';
% input_to_P_lat = '[ K_lat(1)+D_ail; K_lat(2)+D_rud ]';
% input_to_Model_bet = '[ Pilot_ped ]';
% input_to_Model_p = '[ Pilot_lat ]';
% input_to_Perf_bet = '[ Model_bet-P_lat(1) ]';
% input_to_Perf_p = '[ Model_p-P_lat(2) ]';
% input_to_D_ail = '[ Dist_ail ]';
% input_to_D_rud = '[ Dist_rud ]';
% input_to_W_ail = '[ K_lat(1)+D_ail ]';
% input_to_W_rud = '[ K_lat(2)+D_rud ]';
% input_to_N_bet = '[ Noise_bet ]';
% input_to_N_p = '[ Noise_p ]';
% input_to_N_r = '[ Noise_r ]';
% input_to_K_lat = '[ Pilot_lat; Pilot_ped-P_lat(1)-N_bet; P_lat(2)+N_p; P_lat(3)+N_r ]';
% sysoutname = 'CL_lat';
% cleanupsysic = 'yes'; 
% sysic

%% Nonlinear Simulation
vehicle='GTM_T2';
noise_on = 0;
fuelburn_on = 0;

% Construct same doublet sequence via simulink
% set_param('gtm_design/Input Generator/Doublet Generator','timeon','[2 12 7]'); 
% set_param('gtm_design/Input Generator/Doublet Generator','pulsewidth','[0.5 0.5 0.5]');
% set_param('gtm_design/Input Generator/Doublet Generator','amplitude',sprintf('[%f %f %f]',a(1),a(3),a(2)));
[tsim,xsim,ysim]=sim('gtm_design',[0 15]);

% Turn simulink doublet generation off
% set_param('gtm_design/Input Generator/Doublet Generator','amplitude','[0 0 0]');

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
