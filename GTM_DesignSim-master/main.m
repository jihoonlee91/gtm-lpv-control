%%
setup

%% LPV modeling
PitchSurf = 'elev';
verbose = 0;
vabflag = 1;
use_all_inputs = 0;
Ts = 0;

tasGridmps = 40:10:70; % [m/s]
numTAS = numel(tasGridmps);
tasGridkts = convvel(tasGridmps,'m/s','kts'); % [kts]

altGridm = 0:2000:10000; % [m]
numAlt = numel(altGridm);
altGridft = convlength(altGridm,'m','ft'); % [ft]

MWS = grabmws('gtm_design');

isFeasible = zeros(numTAS,numAlt);
for i = 1:numTAS
    for j = 1:numAlt
        targetTASkts = tasGridkts(i);
        targetTASmps = tasGridmps(i);
        target = struct('gamma',0,'tas',targetTASkts);
        targetAltm = altGridm(j);
        targetAltft = altGridft(j);
        MWS.StatesInp(9) = targetAltft;
        loadmws(MWS,'gtm_design');
        [MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose);
        if (Err < 1e-3)
            [sys,londyn,latdyn]=linmodel(MWS,vabflag,use_all_inputs,Ts);
            disp(['TAS ',num2str(targetTASmps),' m/s at altitude ',num2str(targetAltm),' m : Done'])
            isFeasible(i,j) = 1;
        else
            Tc=struct('alpha',NaN,'elevator',NaN,'throttle',NaN);
            disp(['TAS ',num2str(targetTASmps),' m/s at altitude ',num2str(targetAltm),' m : Fail'])
        end
    end
end

%% Longitudinal control design


%% Lateral-directional control design


%% Nonlinear Simulation
vehicle='GTM_T2';
noise_on = 0;
fuelburn_on = 0;
