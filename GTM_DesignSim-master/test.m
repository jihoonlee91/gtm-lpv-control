
%% Component modeling (lateral performance)
% Ideal model
omega_beta = 1;
zeta_beta = 0.7;
omega_phi = 2;
zeta_phi = 0.7;
Ideal_bet = tf(omega_beta^2,[1 2*zeta_beta*omega_beta omega_beta^2]);
Ideal_phi = tf(omega_phi^2,[1 2*zeta_phi*omega_phi omega_phi^2]);

% Weight function
Model_bet = Ideal_bet*Ail*Pade30;
Model_phi = Ideal_phi*Rud*Pade30;
Perf_bet = 2*tf([0.1,5.973],[1,0.176]);
Perf_phi = tf([0.1,5.973],[1,0.176]);
D_ail = 0.1*tf([1,w_ail*10],[1,w_ail]);
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
% control interconnection
systemnames        = 'P_lat Model_bet Model_phi Perf_bet Perf_phi D_ail W_ail W_rud UL_lat Ail Rud Ail_Delay Rud_Delay N_lat DL_lat';
inputvar           = '[bet_c{1};phi_c{1};Dist_ail{1};Noise{5};u{2}]';
outputvar          = '[Perf_bet;Perf_phi;W_ail;W_rud;bet_c;phi_c;DL_lat]';
input_to_UL_lat    = '[u]';
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

%% Lateral LPV synthesis
if isfile('./result/K_lat_lpv.mat') == 0
    tic
    [K_lat_lpv,gamma_lat_lpv,info_lat_lpv] = lpvsyn(G_lat,7,2);
    SynTime_lat_lpv = toc;
    save K_lat_lpv.mat K_lat_lpv gamma_lat_lpv info_lat_lpv SynTime_lat_lpv
else
    disp('Lateral LPV Synthesis Skipped & Data Loaded')
    load K_lat_lpv.mat
end
% disp(['Min Gamma (Lateral Rate-Unbounded LPV) = ',num2str(info_lat_lpv.MinGamma)])
disp(['Gamma (Lateral Rate-Unbounded LPV) = ',num2str(gamma_lat_lpv)])
% disp(['Back-Off Factor (Lateral Rate-Unbounded LPV) = ',num2str((gamma_lat/info_lat_lpv.MinGamma-1)*100),' %'])

%% Lateral rate-bounded LPV synthesis
% G_lat.Domain = DomainRB;
% if isfile('./result/K_lat_rblpv.mat') == 0
%     tic
%     [K_lat_rblpv,gamma_lat_rblpv,info_lat_rblpv] = lpvsyn(G_lat,7,2,Xb,Yb); % Elapsed time is 366 seconds with 4 basis functions
%     SynTime_lat_rblpv = toc;
%     save K_lat_rblpv.mat K_lat_rblpv gamma_lat_rblpv info_lat_rblpv SynTime_lat_rblpv
% else
%     disp('Lateral Rate-Bounded LPV Synthesis Skipped & Data Loaded')
%     load K_lat_rblpv.mat
% end
% G_lat.Domain = Domain;
% % disp(['Min Gamma (Lateral Rate-Bounded LPV) = ',num2str(info_lat_rblpv.MinGamma)])
% disp(['Gamma (Lateral Rate-Bounded LPV) = ',num2str(gamma_lat_rblpv)])
% % disp(['Back-Off Factor (Lateral Rate-Bounded LPV) = ',num2str((gamma_lat_rblpv/info_lat_rblpv.MinGamma-1)*100),' %'])

%% Lateral H_infinity synthesis
if isfile('./result/K_lat_hinf.mat') == 0
    clearvars K_lat_hinfs gamma_lat_hinfs
    for i = 1:num_tas
    for j = 1:num_alt
    for k = 1:num_mor 
        G_lat_LTI = G_lat.value('rho1_tas_param',tas_grid(i),'rho2_alt_param',alt_grid(j),'rho3_mor_param',mor_grid(k));
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
    save K_lat_hinf.mat K_lat_hinf gamma_lat_hinfs SynTime_lat_hinfs
else
    disp('Lateral H_infinity Synthesis Skipped & Data Loaded')
    load K_lat_hinf.mat
end

%% Closed-loop interconnection with Lateral LPV controller and anti-aliasing filter
systemnames        = 'P_lat K_lat_lpv D_ail UL_lat Ail Rud Ail_Delay Rud_Delay DL_lat AA_lat';
inputvar           = '[bet_c{1};phi_c{1};Dist_ail{1}]';
outputvar          = '[P_lat]';
input_to_UL_lat    = '[K_lat_lpv]';
input_to_Ail       = '[UL_lat(1)]';
input_to_Rud       = '[UL_lat(2)]';
input_to_Ail_Delay = '[Ail+D_ail]';
input_to_Rud_Delay = '[Rud]';
input_to_P_lat     = '[Ail_Delay;Rud_Delay]';
input_to_D_ail     = '[Dist_ail]';
input_to_DL_lat    = '[AA_lat]';
input_to_AA_lat    = '[P_lat]';
input_to_K_lat_lpv = '[bet_c;phi_c;P_lat]';
sysoutname         = 'CL_lat_lpv';
cleanupsysic       = 'yes'; 
sysic

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

%% Closed-loop interconnection with Lateral H_infinity controller and anti-aliasing filter
systemnames        = 'P_lat K_lat_hinf D_ail UL_lat Ail Rud Ail_Delay Rud_Delay DL_lat AA_lat';
inputvar           = '[bet_c{1};phi_c{1};Dist_ail{1}]';
outputvar          = '[P_lat]';
input_to_UL_lat    = '[K_lat_hinf]';
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
figure
step(CL_lat_lpv), hold on
step(CL_lat_hinf)
grid on
