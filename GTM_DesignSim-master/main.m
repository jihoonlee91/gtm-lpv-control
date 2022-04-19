%%
close all
clear all
clc

setup
addpath(genpath('./LPVToolsV1.0/'));

%% LPV modeling
PitchSurf = 'elev';
verbose = 1;
vabflag = 1;
use_all_inputs = 0;
Ts = 0;

% tasGridmps = 40:10:70; % [m/s]
tasGridmps = convvel([80], 'kts','m/s'); % [m/s]
numTAS = numel(tasGridmps);
tasGridkts = convvel(tasGridmps,'m/s','kts'); % [kts]
tasParam = pgrid('tas',tasGridkts);

% altGridm = 0:2000:10000; % [m]
altGridm = convlength([800], 'ft','m'); % [m]
numAlt = numel(altGridm);
altGridft = convlength(altGridm,'m','ft'); % [ft]
altParam = pgrid('alt',altGridft);

morphGrid = -12.5; % between -25 and 0
numMORPH = numel(morphGrid);
morphParam = pgrid('morph',morphGrid);

MWS = grabmws('gtm_design');

isFeasible = zeros(numTAS,numAlt);
for i = 1:numTAS
    for j = 1:numAlt
        for k = 1:numAlt
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
            if (Err < 1e-3)
                [sys,londyn,latdyn]=linmodel(MWS,vabflag,use_all_inputs,Ts);
                disp(['TAS ',num2str(targetTASmps),' m/s at altitude ',num2str(targetAltm),' m : Done'])
                isFeasible(i,j,k) = 1;
            else
                Tc=struct('alpha',NaN,'elevator',NaN,'throttle',NaN);
                disp(['TAS ',num2str(targetTASmps),' m/s at altitude ',num2str(targetAltm),' m : Fail'])
            end
            Glon(:,:,i,j,k) = londyn;
            Glat(:,:,i,j,k) = latdyn;
        end
    end
end

Domain = rgrid(tasParam,altParam,morphParam);
lpvLon = pss(Glon,Domain);
lpvLat = pss(Glat,Domain);

%% Longitudinal control design
% Define weights:
We = tf(10,[1,0.1]);
Wu = 1/20;
Wa = 1/20;
% tf([1,0.4],[1,4000]);
% Wn = blkdiag(Wn1,Wn2,Wn3);
Wn = diag([0.1,0.1,0.2]);

wa = 105;
za = 0.6;
act = ss([0,1;-wa^2,-2*za*wa],[0;wa^2],[1,0],0);

% wr = 2*pi;
% zr = 0.7;

% wr = 2*pi*(2*eta1+1);
% zr = eta2

[freq,zeta] = damp(sys);
wr = freq(1);
zr = zeta(1);

Wr = ss([0,1;-wr^2,-2*zr*wr],[0;wr^2],[1,0],0);

% Form synthesis interconnection:
systemnames = 'sys act We Wu Wa Wn Wr';
inputvar = '[ref{1};dn{3};u]';
outputvar = '[Wu;Wa;We;ref;sys+Wn]';
input_to_sys = '[act]';
input_to_act = '[u]';
input_to_Wa = '[act]';
input_to_Wu = '[u]';
input_to_Wn = '[dn]';
input_to_We = '[sys(1)-Wr]';
input_to_Wr = '[ref]';
G = sysic;

% Synthesize two-degree of freedom controller.
b0 = basis(1,0);
b1 = basis(eta1,'eta1',1);
b2 = basis(eta2,'eta2',1);
Xb = [b0;b1;b1^2;b2;b2^2;b1*b2];
Yb = Xb;
opt = lpvsynOptions('BackOffFactor',1.02);
nmeas = 4; % # of measurements
ncont = 1; % # of controls
tic
[Knr,Gamma,Info] = lpvsyn(G,nmeas,ncont,Xb,Yb,opt);
toc
Gamma

% Insert Knr into the weighted interconnection:
CLICnr = lft(G,Knr);

% Form closed-loop sytem
systemnames = 'sys act Knr';
inputvar = '[r{1}]';
outputvar = '[sys]';
input_to_sys = '[act]';
input_to_act = '[Knr]';
input_to_Knr = '[r;sys]';
CL = sysic;


%% Lateral-directional control design


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
ysim=ysim(:,7:18);

%% Plot results
set(figure(1),'Position',[20 80 900 700]);
subplot(321),
plot(tsim,ysim(:,1)); grid on
title('Linear Velocity to Doublet Sequence [elevator,rudder,aileron]');
xlabel('Time (sec)');ylabel('u (ft/sec)')
legend('Simulation', 'Linear');
subplot(323),
plot(tsim,ysim(:,2)); grid on
xlabel('Time (sec)');ylabel('v (ft/sec)')
subplot(325),
plot(tsim,ysim(:,3)); grid on
xlabel('Time (sec)');ylabel('w (ft/sec)')

subplot(322),
plot(tsim,180/pi*ysim(:,4)); grid on
title('Angular Velocity to Doublet Sequence [elevator,rudder,aileron]');
xlabel('Time (sec)');ylabel('p (deg/sec)')
subplot(324),
plot(tsim,180/pi*ysim(:,5)); grid on
xlabel('Time (sec)');ylabel('q (deg/sec)')
subplot(326),
plot(tsim,180/pi*ysim(:,6)); grid on
xlabel('Time (sec)');ylabel('r (deg/sec)')
