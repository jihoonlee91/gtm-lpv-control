%% Component modeling (lateral performance)
% Ideal model
omega_beta = 1;
zeta_beta = 0.7;
omega_phi = 2.8;
zeta_phi = 0.7;
Ideal_bet = tf(omega_beta^2,[1 2*zeta_beta*omega_beta omega_beta^2]);
Ideal_phi = tf(omega_phi^2,[1 2*zeta_phi*omega_phi omega_phi^2]);

% Weight function
Model_bet = Ideal_bet*Ail*Pade30;
Model_phi = Ideal_phi*Rud*Pade30;
Perf_bet = tf([0.1,5.973],[1,0.176]);
Perf_phi = tf([0.1,5.973],[1,0.176]);
D_ail = 0.25*tf([1,w_ail*10],[1,w_ail]);
W_ail = 0.8*tf([1,1.5],[1,4]);
W_rud = 0.8*tf([100,53.64],[1,119.2]);
N_bet = 0.033*pi/180*(s+10)/(s+100); % 0.033 deg
N_p = 0.522*pi/180*(s+10)/(s+100); % 0.522 deg/s
N_r = 0.333*pi/180*(s+10)/(s+100); % 0.333 deg/s
N_phi = 0.02*pi/180*(s+10)/(s+100); % 0.02 deg
N_Ay = 0.0082*g*(s+10)/(s+100); % 0.0082 g

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

%% Lateral LPV modeling
P_lat = pss(GTM_lat,Domain);

% control interconnection
systemnames        = 'P_lat Model_bet Model_phi Perf_bet Perf_phi D_ail W_ail W_rud UL_lat Ail Rud Ail_Delay Rud_Delay N_lat DL_lat';
inputvar           = '[bet_c{1};phi_c{1};Dist_ail{1};Noise{5};u{2}]';
outputvar          = '[Perf_bet;Perf_phi;W_ail;W_rud;bet_c;phi_c;DL_lat]';
input_to_UL_lat    = '[u(1);u(2)]';
input_to_Ail       = '[UL_lat(1)]';
input_to_Rud       = '[UL_lat(2)]';
input_to_Ail_Delay = '[Ail+D_ail]';
input_to_Rud_Delay = '[Rud]';
input_to_P_lat     = '[Ail_Delay;Rud_Delay]';
input_to_Model_bet = '[bet_c]';
input_to_Model_phi = '[phi_c]';
input_to_Perf_bet  = '[Model_bet-P_lat(1)]';
input_to_Perf_phi  = '[Model_phi-P_lat(4)]';
input_to_D_ail     = '[Dist_ail]';
input_to_W_ail     = '[u(1)]';
input_to_W_rud     = '[u(2)]';
input_to_N_lat     = '[Noise]';
input_to_DL_lat    = '[P_lat+Noise]';
sysoutname         = 'G_lat';
cleanupsysic       = 'yes'; 
sysic

%% Lateral H_infinity synthesis
% if isfile('K_lat_hinf.mat') == 0
    clearvars K_lat_hinfs gamma_lat_hinfs
    for i = 1:num_tas
    for j = 1:num_alt
    for k = 1:num_mor 
        G_lat_LTI = G_lat.value('tas_param',tas_grid(i),'alt_param',alt_grid(j),'mor_param',mor_grid(k));
        tic
        [K_lat_hinf_point,~,gamma_lat_hinf_point] = hinfsyn(G_lat_LTI,7,2);
        SynTime_lat_hinfs(i,j,k) = toc;
        disp(['Gamma (Lateral H_infinity) = ',num2str(gamma_lat_hinf_point)])
        gamma_lat_hinfs(i,j,k) = gamma_lat_hinf_point;
        K_lat_hinfs(:,:,i,j,k) = K_lat_hinf_point;
    end
    end
    end
    K_lat_hinf = pss(K_lat_hinfs,Domain);
%     save K_lat_hinf.mat K_lat_hinf gamma_lat_hinfs SynTime_lat_hinfs
% else
%     disp('Lateral H_infinity Synthesis Skipped & Data Loaded')
%     load K_lat_hinf.mat
% end

%% Closed-loop interconnection with Lateral H_infinity controller and anti-aliasing filter
systemnames        = 'P_lat K_lat_hinf D_ail UL_lat Ail Rud Ail_Delay Rud_Delay DL_lat AA_lat';
inputvar           = '[bet_c{1};phi_c{1};Dist_ail{1}]';
outputvar          = '[P_lat]';
input_to_UL_lat    = '[K_lat_hinf(1);K_lat_hinf(2)]';
input_to_Ail       = '[UL_lat(1)]';
input_to_Rud       = '[UL_lat(2)]';
input_to_Ail_Delay = '[Ail+D_ail]';
input_to_Rud_Delay = '[Rud]';
input_to_P_lat     = '[Ail_Delay;Rud_Delay]';
input_to_D_ail     = '[Dist_ail]';
input_to_DL_lat    = '[AA_lat]';
input_to_AA_lat    = '[P_lat]';
input_to_K_lat_hinf= '[bet_c;phi_c;P_lat]';
sysoutname         = 'CL_lat_hinf';
cleanupsysic       = 'yes'; 
sysic

%% Closed-loop comparison
step(CL_lat_hinf)
grid on

% Li = K_lat_hinf*P_lat;