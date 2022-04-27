%% Initialization
close all
clear all
clc

setup
addpath(genpath('./LPVToolsV1.0/'));

%% Trimming
PitchSurf = 'elev';
verbose = 1;
vabflag = 1;
use_all_inputs = 0;
Ts = 0;

% tasGridmps = convvel([70 85], 'kts','m/s'); % [m/s]
tasGridmps = convvel(80, 'kts','m/s'); % [m/s]
numTAS = numel(tasGridmps);
tasGridkts = convvel(tasGridmps,'m/s','kts'); % 70~85 [kts]
tasParam = pgrid('tasParam',tasGridkts);
tasParam.RateBounds = 10*[-1,1];

% altGridm = convlength([700 1100], 'ft','m'); % [m]
altGridm = convlength(1000, 'ft','m'); % [m]
numAlt = numel(altGridm);
altGridft = convlength(altGridm,'m','ft'); % 700~1100 [ft]
altParam = pgrid('altParam',altGridft);
altParam.RateBounds = 100*[-1,1];

% morphGrid = [-25 0]; % between -25 and 0
morphGrid = 0; % between -25 and 0
numMorph = numel(morphGrid);
morphParam = pgrid('morphParam',morphGrid);
morphParam.RateBounds = 10*[-1,1];

MWS = grabmws('gtm_design');

isFeasible = zeros(numTAS,numAlt);
for i = 1:numTAS
    for j = 1:numAlt
        for k = 1:numMorph
            targetTASkts = tasGridkts(i);
            targetTASmps = tasGridmps(j);
            targetMORPH = morphGrid(k);

            target = struct('gamma',0,'tas',targetTASkts);

            targetAltm = altGridm(j);
            targetAltft = altGridft(j);
            MWS.StatesInp(9) = targetAltft;
            
            MWS.bias.morphing = targetMORPH;

            loadmws(MWS,'gtm_design');
            [MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose);
            mwsGrid(i,j,k) = MWS;
            if (Err < 1e-3)
                disp(['TAS ',num2str(targetTASmps),' m/s at altitude ',num2str(targetAltm),' m : Done'])
                isFeasible(i,j,k) = 1;
            else
                Tc=struct('alpha',NaN,'elevator',NaN,'throttle',NaN);
                disp(['TAS ',num2str(targetTASmps),' m/s at altitude ',num2str(targetAltm),' m : Fail'])
            end
        end
    end
end

%% Linearization
for i = 1:numTAS
    for j = 1:numAlt
        for k = 1:numMorph
            if isFeasible(i,j,k) == 1
                [sys,londyn,latdyn]=linmodel(mwsGrid(i,j,k),vabflag,use_all_inputs,Ts);
                GTM_lon(:,:,i,j,k) = ss(londyn.A,londyn.B(:,1),londyn.C([2,3],:),londyn.D([2,3],1));
                GTM_lat(:,:,i,j,k) = ss(latdyn.A,latdyn.B(:,[1,2]),latdyn.C([1,2,3],:),latdyn.D([1,2,3],[1,2]));
            end
        end
    end
end

%% LPV modeling
Domain = rgrid(tasParam,altParam,morphParam);
P_lon = pss(GTM_lon,Domain);
P_lat = pss(GTM_lat,Domain);

%% Longitudinal control design (baseline controller)
% Weights
Model_alp = tf(16,[1,5.6,16])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
Perf_alp = tf([0.1,5.973],[1,0.176]);
D_elev = 0.01;
W_elev = tf([0.8,42.91],[1,119.2]);
N_alp = 0.01;
N_q = 0.1;

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
G_lon

%% Longitudinal synthesis
% Basis function
b1 = basis(1,0);
btas = basis(tasParam,'tasParam',1);
balt = basis(altParam,'altParam',1);
bmorph = basis(morphParam,'morphParam',1);
Xb = [b1;btas;balt;bmorph];
Yb = Xb;

% Synthesize two-degree of freedom controller.
opt = lpvsynOptions('BackOffFactor',1.02);
nmeas = 2; % # of measurements
ncont = 1; % # of controls
[K_lon,gamma_lon,info_lon] = lpvsyn(G_lon,nmeas,ncont,Xb,Yb,opt);
K_lon
gamma_lon
info_lon

%% Analysis
% Closed-loop interconnection
systemnames = ' P_lon Model_alp Perf_alp W_elev D_elev N_alp N_q K_lon ';
inputvar = '[ Pilot_lon{1}; Dist_elev{1}; Noise_alp{1}; Noise_q{1} ]';
outputvar = '[ Perf_alp; W_elev ]';
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
CL_lon

%% Lateral control design (baseline controller)
% Weights
% s = tf('s');
Model_bet = tf(7.78,[1,3.92,7.78])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
Model_p = tf(-52.56,[1,0.04])*tf([1,-0.02],[1,6.47])*tf([1,7.5,34.67],[1,8.75,39.06])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
Perf_bet = tf([0.1,5.973],[1,0.176]);
Perf_p = tf([0.1,5.973],[1,0.299]);
D_ail = 0.01;
D_rud = 0.01;
W_ail = tf([0.8,42.91],[1,119.2]);
W_rud = tf([0.8,42.91],[1,119.2]);
N_bet = 0.01;
N_p = 0.1;
N_r = 0.1;

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
G_lat

%% Lateral synthesis
% Basis function
b1 = basis(1,0);
btas = basis(tasParam,'tasParam',1);
balt = basis(altParam,'altParam',1);
bmorph = basis(morphParam,'morphParam',1);
Xb = [b1;btas;balt;bmorph];
Yb = Xb;

% Synthesize two-degree of freedom controller.
opt = lpvsynOptions('BackOffFactor',1.02);
nmeas = 3; % # of measurements
ncont = 2; % # of controls
[K_lat,gamma_lat,info_lat] = lpvsyn(G_lat,nmeas,ncont,Xb,Yb,opt);
K_lat
gamma_lat
info_lat

%% Analysis
% K_lat = rss(18,2,4);

% Closed-loop interconnection
systemnames = ' P_lat Model_bet Model_p Perf_bet Perf_p D_ail D_rud W_ail W_rud N_bet N_p N_r K_lat ';
inputvar = '[ Pilot_lat{1}; Pilot_ped{1}; Dist_ail{1}; Dist_rud{1}; Noise_bet{1}; Noise_p{1}; Noise_r{1} ]';
outputvar = '[ Perf_bet; Perf_p; W_ail; W_rud ]';
input_to_P_lat = '[ K_lat(1)+D_ail; K_lat(2)+D_rud ]';
input_to_Model_bet = '[ Pilot_ped ]';
input_to_Model_p = '[ Pilot_lat ]';
input_to_Perf_bet = '[ Model_bet-P_lat(1) ]';
input_to_Perf_p = '[ Model_p-P_lat(2) ]';
input_to_D_ail = '[ Dist_ail ]';
input_to_D_rud = '[ Dist_rud ]';
input_to_W_ail = '[ K_lat(1)+D_ail ]';
input_to_W_rud = '[ K_lat(2)+D_rud ]';
input_to_N_bet = '[ Noise_bet ]';
input_to_N_p = '[ Noise_p ]';
input_to_N_r = '[ Noise_r ]';
input_to_K_lat = '[ Pilot_lat; Pilot_ped-P_lat(1)-N_bet; P_lat(2)+N_p; P_lat(3)+N_r ]';
sysoutname = 'CL_lat';
cleanupsysic = 'yes'; 
sysic
CL_lat
