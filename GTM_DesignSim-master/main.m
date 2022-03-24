%%
setup

%% LPV modeling
PitchSurf = 'elev';
verbose = 1;
vabflag = 1;
use_all_inputs = 0;
Ts = 0;

% tasGridmps = 40:10:70; % [m/s]
tasGridmps = 50; % [m/s]
numTAS = numel(tasGridmps);
tasGridkts = convvel(tasGridmps,'m/s','kts'); % [kts]

% altGridm = 0:2000:10000; % [m]
altGridm = 0; % [m]
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

% Construct 1 degree double sequence. 
f=100;    % 100 Hz input sampling on sequence
d=0.5;      % 2 sec pulse duration
a=[1 1 1];% pulse amplitude(deg), [ele,rud,ail]

dub=[zeros(4*f*d,1);ones(f*d,1);-1*ones(f*d,1);zeros(4*f*d,1)]; % a doublet
ulin = [  [a(1)*dub;0*dub;0*dub], [0*dub;0*dub;a(3)*dub], [0*dub;a(2)*dub;0*dub] ]; % doublet sequence
tlin=[0:length(ulin)-1]/f;

% Construct same doublet sequence via simulink
set_param('gtm_design/Input Generator/Doublet Generator','timeon','[2 12 7]'); 
set_param('gtm_design/Input Generator/Doublet Generator','pulsewidth','[0.5 0.5 0.5]');
set_param('gtm_design/Input Generator/Doublet Generator','amplitude',sprintf('[%f %f %f]',a(1),a(3),a(2)));
[tsim,xsim,ysim]=sim('gtm_design',[0 max(tlin)]);

% Turn simulink doublet generation off
set_param('gtm_design/Input Generator/Doublet Generator','amplitude','[0 0 0]');

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
