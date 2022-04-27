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
                Glon(:,:,i,j,k) = ss(londyn.A,londyn.B(:,1),londyn.C([2,3],:),londyn.D([2,3],1));
                Glat(:,:,i,j,k) = ss(latdyn.A,latdyn.B(:,[1,2]),latdyn.C([1,2,3],:),latdyn.D([1,2,3],[1,2]));
            end
        end
    end
end

%% LPV modeling
Domain = rgrid(tasParam,altParam,morphParam);
G = pss(Glon,Domain);
GTMlat = pss(Glat,Domain);

%% Longitudinal control design (baseline controller)
% Weights
M = tf(16,[1,5.6,16])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
P = tf([0.1,5.973],[1,0.176]);
% D = 0.01;
W = tf([0.8,42.91],[1,119.2]);
% Nq = 0.1;
% Na = 0.01;

M = tf(1,[1,0]);
P = tf(1,[1,0]);
% D = 0.01;
W = tf(1,[1,0]);

% Control interconnection
systemnames = ' G M P W ';
inputvar = '[ p{1}; d{1}; na{1}; nq{1}; u ]';
outputvar = '[ P; W; p-G(1)-0.01*na; G(2)+0.1*nq ]';
input_to_G = '[ u+0.01*d ]';
input_to_M = '[ p ]';
input_to_P = '[ M-G(1) ]';
input_to_W = '[ u+0.01*d ]';
sysoutname = 'H';
cleanupsysic = 'yes'; 
sysic

%%
% Control interconnection (closed-loop)
% systemnames = ' GTMlon Modela Perfa Delev Welev Na Nq K ';
% inputvar = '[ Pilotlon{1}; Distelev{1}; Noisea{1}; Noiseq{1} ]';
% outputvar = '[ Perfa; Welev ]';
% input_to_GTMlon = '[ K+Delev ]';
% input_to_Modela = '[ Pilotlon ]';
% input_to_Perfa = '[ Modela-GTMlon(1) ]';
% input_to_Delev = '[ Distelev ]';
% input_to_Welev = '[ K+Delev ]';
% input_to_Na = '[ Noisea ]';
% input_to_Nq = '[ Noiseq ]';
% input_to_K = '[ Pilotlon-GTMlon(1)-Na; GTMlon(2)+Nq ]';
% sysoutname = 'H';
% cleanupsysic = 'yes'; 
% sysic

%% Longitudinal synthesis
% Basis function,
% b1 = basis(1,0);
% bcos = basis(pcos,'rho',-psin);
% bsin = basis(psin,'rho',pcos);
% Xb = [b1;bcos;bsin];
% Yb = Xb;
% 
% % Synthesize two-degree of freedom controller.
% b0 = basis(1,0);
% b1 = basis(eta1,'eta1',1);
% b2 = basis(eta2,'eta2',1);
% Xb = [b0;b1;b1^2;b2;b2^2;b1*b2];
% Yb = Xb;
% opt = lpvsynOptions('BackOffFactor',1.02);
% nmeas = 4; % # of measurements
% ncont = 1; % # of controls
% 
% [Knr,Gamma,Info] = lpvsyn(G,nmeas,ncont,Xb,Yb,opt);
% 
% % LPV Rate-Bounded Control Design
% opt = lpvsynOptions('BackOffFactor',1.02);
% [Klpv,normlpv] = lpvsyn(H,2,2,Xb,Yb,opt);