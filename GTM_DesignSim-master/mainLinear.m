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
tasParam = pgrid('tas',tasGridkts);

% altGridm = convlength([700 1100], 'ft','m'); % [m]
altGridm = convlength(800, 'ft','m'); % [m]
numAlt = numel(altGridm);
altGridft = convlength(altGridm,'m','ft'); % 700~1100 [ft]
altParam = pgrid('alt',altGridft);

% morphGrid = [-25 0]; % between -25 and 0
morphGrid = 0; % between -25 and 0
numMorph = numel(morphGrid);
morphParam = pgrid('morph',morphGrid);

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
                Glon(:,:,i,j,k) = londyn;
                Glat(:,:,i,j,k) = latdyn;
            end
        end
    end
end

%% LPV modeling
Domain = rgrid(tasParam,altParam,morphParam);
GTM_lon = pss(Glon,Domain);
GTM_lat = pss(Glat,Domain);

%% Longitudinal control design (baseline controller)
% Weights
Model_a = tf(16,[1,5.6,16])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
Perf_a = tf([0.1,5.973],[1,0.176]);
D_elev = 0.01;
W_elev = tf([0.8,42.91],[1,119.2]);
N_q = 0.1;
N_a = 0.01;

% Control interconnection
systemnames = 'GTM_lon Model_a Perf_a D_elev W_elev N_q N_a';
input_to_GTM_lon = '[u+D_elev]';
input_to_Model_a = '[Pilot_lon]';
input_to_Perf_a = '[Model_a-GTM_lon(2)]';
input_to_D_elev = '[Dist_elev]';
input_to_W_elev = '[u+D_elev]';
input_to_N_q = '[Noise_q]';
input_to_N_a = '[Noise_a]';
inputvar = '[Pilot_lon{1};Noise_q{1};Noise_a{1};u{1}]';
outputvar = '[Perf_a;W_elev;Pilot_lon-GTM_lon(2)-N_a;GTM_lon(3)+N_q]';
H = sysic;

% Basis function,
b1 = basis(1,0);
bcos = basis(pcos,'rho',-psin);
bsin = basis(psin,'rho',pcos);
Xb = [b1;bcos;bsin];
Yb = Xb;

% LPV Rate-Bounded Control Design
opt = lpvsynOptions('BackOffFactor',1.02);
[Klpv,normlpv] = lpvsyn(H,2,2,Xb,Yb,opt);