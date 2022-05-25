%% Initialization
close all
clear all
clc

setup
addpath(genpath('./LPVToolsV1.0/'));

MWS = grabmws('gtm_design');

%% Setting
PitchSurf = 'elev'; % pitch surface
verbose = 1; % trim output display off
vabflag = 1; % wind-axis linear state
use_all_inputs = 0; % control surface as groups
Ts = 0; % continuous-time model
rateBound = 0; % rate-unbounded
LPVControlOn = 0;
RateBoundOn = 0;

% Parameter grid
tas_grid = [100]; % 70~85 [kts]
alt_grid = [1000]; % 700~1100 [ft]
mor_grid = [0]; % between -25 and 0 [%]
num_tas = numel(tas_grid);
num_alt = numel(alt_grid);
num_mor = numel(mor_grid);

% Parameters
tas_param = pgrid('tas_param',tas_grid);
alt_param = pgrid('alt_param',alt_grid);
mor_param = pgrid('mor_param',mor_grid);

% Rate bounds
if rateBound == 1
    tas_rbound = 60; % [kts/s]
    alt_rbound = 25; % [ft/s]
    mor_rbound = 10; % [%/s]
    tas_param.RateBounds = [-1,1]*tas_rbound;
    alt_param.RateBounds = [-1,1]*alt_rbound;
    mor_param.RateBounds = [-1,1]*mor_rbound;
end

Pilot_lon = 0;
Pilot_lat = 0;
Pilot_ped = 0;

%% Trimming
for i = 1:num_tas
    for j = 1:num_alt
        for k = 1:num_mor
            trim_point = (i-1)*num_alt*num_mor + (j-1)*num_mor + k;

            tas = tas_grid(i);
            alt = alt_grid(j);
            mor = mor_grid(k);

            target = struct('gamma',0,'tas',tas);
            MWS.StatesInp(9) = alt;
            MWS.bias.morphing = mor;

            loadmws(MWS,'gtm_design');

            [MWS,Xtrim,Trimcond,Err] = trimgtm(target,PitchSurf,verbose);
            MWS_grid(i,j,k) = MWS;
            tas_trim(i,j,k) = Trimcond.tas;
            alt_trim(i,j,k) = Trimcond.altitude;
            mor_trim(i,j,k) = mor;
            alp_trim(i,j,k) = Trimcond.tas;
            throttle_trim(i,j,k) = Trimcond.throttle;
            aileron_trim(i,j,k) = Trimcond.aileron;
            elevator_trim(i,j,k) = Trimcond.elevator;
            rudder_trim(i,j,k) = Trimcond.rudder;

            if (Err < 1e-3)
%                 disp(['TRIM @ ',num2str(trim_point),'/',num2str(num_tas*num_alt*num_mor),' : DONE'])
            else
%                 Trimcond=struct('alpha',NaN,'elevator',NaN,'throttle',NaN);
                error(['TRIM @ ',num2str(trim_point),'/',num2str(num_tas*num_alt*num_mor),' : FAIL'])
            end
            disp('-------------------------------------------------------------------')
        end
    end
end

%% Linearization
f_ele = 5; % [Hz]
f_ail = 5; % [Hz]
f_rud = 5; % [Hz]

for i = 1:num_tas
    for j = 1:num_alt
        for k = 1:num_mor
            trim_point = (i-1)*num_alt*num_mor + (j-1)*num_mor + k;
            
            % linearization
            [gtmdyn,londyn,latdyn]=linmodel(MWS_grid(i,j,k),vabflag,use_all_inputs,Ts);
            londyn
            latdyn

            V0 = tas_grid(i);
            Z_dele = londyn.B(2,1)*V0;
            Z_alp = londyn.A(2,2)*V0;
            M_dele = londyn.B(3,1);
            M_alp = londyn.A(3,2);
            
            % Augmentation ()
            Alon = [londyn.A(1,:);
                londyn.A(4,:)-londyn.A(2,:);
                londyn.A(3:4,:)]; % V gamma q theta
            Blon = [londyn.B(1,4)+londyn.B(1,5),londyn.B(1,1);
                -londyn.B(2,4)-londyn.B(2,5),-londyn.B(2,1)
                londyn.B(3:4,4)+londyn.B(3:4,5),londyn.B(3:4,1)]; % delt delec
            Clon = eye(4); % V gamma q theta
            Dlon = zeros(4,2);
            londyn_aug = ss(Alon,Blon,Clon,Dlon);
            set(londyn_aug,'Statename',{'tas','gamma','q','theta'},...
                           'Inputname',{'delt','delec'},...
                           'Outputname',{'tas','gamma','q','theta'})

            GTM_lon(:,:,i,j,k) = londyn_aug;

%             GTM_lon(:,:,i,j,k) = ss(londyn.A,londyn.B(:,1),londyn.C(2:3,:),londyn.D(2:3,1));
%             GTM_lon(:,:,i,j,k).StateName = londyn.StateName;
%             GTM_lon(:,:,i,j,k).InputName = londyn.InputName(1);
%             GTM_lon(:,:,i,j,k).OutputName = londyn.OutputName(2:3);

            GTM_lat(:,:,i,j,k) = ss(latdyn.A,latdyn.B(:,[1,2]),latdyn.C(1:3,:),latdyn.D(1:3,1:2));
            GTM_lat(:,:,i,j,k).StateName = latdyn.StateName;
            GTM_lat(:,:,i,j,k).InputName = latdyn.InputName(1:2);
            GTM_lat(:,:,i,j,k).OutputName = latdyn.OutputName(1:3);

            figure, bode(londyn), grid on, title('Longitudinal Dynamics')
%             figure, bode(latdyn), grid on, title('Lateral Dynamics')

            disp('-------------------------------------------------------------------')
        end
    end
end

nmeas_lon = 4; % # of measurements
ncont_lon = 2; % # of controls

nmeas_lat = 4; % # of measurements
ncont_lat = 2; % # of controls

%% LPV modeling
Domain = rgrid(tas_param,alt_param,mor_param);
P_lon = pss(GTM_lon,Domain);
P_lat = pss(GTM_lat,Domain);

%% Longitudinal H-infinity control design (augmented system)
s = tf('s');
% Weight function design
T_id = diag([tf(0.35^2,[1,2*1*0.35,0.35^2]),tf(0.15^2,[1,2*1*0.15,0.15^2])]);
Model_gam = ;
Wp = dig([100*(s/100+1)/(s/0.005+1),100*(s/100+1)/(s/0.005+1)]);
Wr = diag([10,3*pi/180]);
Wn = diag([0.05*(s/0.5+1)/(s/100+1),0.01,0.05*(s/0.5+1)/(s/100+1),0.05*(s/0.5+1)/(s/100+1)]);

for i = 1:num_tas
    for j = 1:num_alt
        for k = 1:num_mor
            trim_point = (i-1)*num_alt*num_mor + (j-1)*num_mor + k;
            
            LTI_lon = GTM_lon(:,:,i,j,k);
            
            % LTI control interconnection
            systemnames = ' LTI_lon W_elev Model_tas Model_alt Perf_tas Perf_alt';
            inputvar = '[ tas_c{1}; alt_c{1}; u{2} ]';
            outputvar = '[ W_elev; tas_c-LTI_lon(1); alt_c-LTI_lon(5); LTI_lon(2); LTI_lon(3); LTI_lon(4); LTI_lon(6); LTI_lon(7) ]';
            input_to_LTI_lon = '[ u ]';
            input_to_Model_tas = '[ tas_c ]';
            input_to_Model_alt = '[ alt_c ]';
            input_to_Perf_tas = '[ Model_tas-LTI_lon(1) ]';
            input_to_Perf_alt = '[ Model_alt-LTI_lon(5) ]';
%             input_to_D_elev = '[ Dist_elev ]';
            input_to_W_elev = '[ u ]';
%             input_to_N_alp = '[ Noise_alp ]';
%             input_to_N_q = '[ Noise_q ]';
            sysoutname = 'LTI_G_lon';
            cleanupsysic = 'yes'; 
            sysic
            
            % Synthesis
            [K_lon_hinf,~,gamma_lon_hinfs(i,j,k)] = hinfsyn(LTI_G_lon,nmeas_lon,ncont_lon);
            disp(['H-Infinity Synthesis DONE'])
            gamma_lon_hinfs
            K_lon_hinfs(:,:,i,j,k) = K_lon_hinf;
            
%             Closed-loop interconnection
            systemnames = ' LTI_lon K_lon_hinf ';
            inputvar = '[ tas_c{1}; alt_c{1} ]';
            outputvar = '[ LTI_lon(1); LTI_lon(5) ]';
            input_to_LTI_lon = '[ K_lon_hinf ]';
            input_to_K_lon_hinf = '[ tas_c-LTI_lon(1); alt_c-LTI_lon(5); LTI_lon(2); LTI_lon(3); LTI_lon(4); LTI_lon(6); LTI_lon(7) ]';
            sysoutname = 'CL_lon_hinf';
            cleanupsysic = 'yes'; 
            sysic

            CL_lon_hinfs(:,:,i,j,k) = CL_lon_hinf;
        end
    end
end
figure, bode(CL_lon_hinf), grid on
figure, step(CL_lon_hinf,[0,200])
grid on