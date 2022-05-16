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
tas_grid = [80]; % 70~85 [kts]
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

clearvars GTM_lon GTM_lat
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
%             M_dele = londyn.B(3,1);
%             M_alp = londyn.A(3,2);
            
            % Augmentation (altitude & first-order elevator dynamics)
%             Alon = londyn.A;
%             Blon = [londyn.B(:,4)+londyn.B(:,5),londyn.B(:,1)]; % delt delec
%             Clon = eye(4);
%             Dlon = zeros(4,2);
%             londyn_aug = ss(Alon,Blon,Clon,Dlon);
%             set(londyn_aug,'Statename',{'tas','alpha','q','theta'},...
%                            'Inputname',{'delt','dele'},...
%                            'Outputname',{'tas','alpha','q','theta'})

            % Augmentation (altitude & first-order elevator dynamics)
            Alon = [londyn.A,zeros(4,1);[0,-V0,0,V0],0]; % V alp q theta h
            Blon = [londyn.B(:,4)+londyn.B(:,5),londyn.B(:,1);
                zeros(1,2)]; % delt delec
            Clon = [eye(5);
                Alon(1,:)
                [0,Z_alp,0,0,0]]; % V alp q theta h Ax Az
            Dlon = [zeros(6,2);
                0,Z_dele];
            londyn_aug = ss(Alon,Blon,Clon,Dlon);
            set(londyn_aug,'Statename',{'tas','alpha','q','theta','alt'},...
                           'Inputname',{'delt','dele'},...
                           'Outputname',{'tas','alpha','q','theta','alt','Ax','Az'})

            % Augmentation (altitude & first-order elevator dynamics)
%             Alon = [londyn.A,zeros(4,1),londyn.B(:,1);[0,-V0,0,V0;zeros(1,4)],[zeros(1,2);0,-2*pi*f_ele]]; % V alp q theta h dele
%             Blon = [[londyn.B(:,4)+londyn.B(:,5);zeros(2,1)],[zeros(5,1);2*pi*f_ele]]; % delt delec
%             Clon = [eye(6);[0,Z_alp,0,0,0,Z_dele]]; % V alp q theta h dele Az
%             Dlon = zeros(7,2);
%             londyn_aug = ss(Alon,Blon,Clon,Dlon);
%             set(londyn_aug,'Statename',{'tas','alpha','q','theta','alt','dele'},...
%                            'Inputname',{'delt','delec'},...
%                            'Outputname',{'tas','alpha','q','theta','alt','dele','Az'})

            GTM_lon(:,:,i,j,k) = londyn_aug;

%             GTM_lon(:,:,i,j,k) = ss(londyn.A,londyn.B(:,1),londyn.C(2:3,:),londyn.D(2:3,1));
%             GTM_lon(:,:,i,j,k).StateName = londyn.StateName;
%             GTM_lon(:,:,i,j,k).InputName = londyn.InputName(1);
%             GTM_lon(:,:,i,j,k).OutputName = londyn.OutputName(2:3);

            GTM_lat(:,:,i,j,k) = ss(latdyn.A,latdyn.B(:,[1,2]),latdyn.C(1:3,:),latdyn.D(1:3,1:2));
            GTM_lat(:,:,i,j,k).StateName = latdyn.StateName;
            GTM_lat(:,:,i,j,k).InputName = latdyn.InputName(1:2);
            GTM_lat(:,:,i,j,k).OutputName = latdyn.OutputName(1:3);

%             figure, bode(londyn_aug), grid on, title('Longitudinal Dynamics')
%             figure, bode(latdyn_aug), grid on, title('Lateral Dynamics')

            disp('-------------------------------------------------------------------')
        end
    end
end

nmeas_lon = 7; % # of measurements
ncont_lon = 2; % # of controls

nmeas_lat = 4; % # of measurements
ncont_lat = 2; % # of controls

%% LPV modeling
Domain = rgrid(tas_param,alt_param,mor_param);
P_lon = pss(GTM_lon,Domain);
P_lat = pss(GTM_lat,Domain);

%% Longitudinal H-infinity control design
s = tf('s');
% Weight function design
Model_tas = (0.15^2)/(s^2+2*1*0.15+0.15^2);
Model_alt = tf(16,[1,5.6,16])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);

Perf_tas = 100*(s/100+1)/(s/0.005+1);
Perf_alt = tf([0.1,5.973],[1,0.176]);

D_thro = 0.01;
D_elev = 0.01;

W_thro = 0.1*(s/0.5+1)/(s/100+1);
W_elev = 0.001;

N_tas = 0.01;
N_alp = 0.01;
N_q = 0.01;
N_theta = 0.01;
N_alt = 0.01;
N_Ax = 0.01;
N_Az = 0.01;

tic
for i = 1:num_tas
    for j = 1:num_alt
        for k = 1:num_mor
            trim_point = (i-1)*num_alt*num_mor + (j-1)*num_mor + k;
            
            LTI_lon = GTM_lon(:,:,i,j,k);
            
            % LTI control interconnection
            systemnames = ' LTI_lon Model_tas Model_alt Perf_tas Perf_alt D_thro D_elev W_thro W_elev N_tas N_alp N_q N_theta N_alt N_Ax N_Az';
            inputvar = '[ tas_c{1}; alt_c{1}; Dist_thro{1}; Dist_elev{1}; Noise_tas{1}; Noise_alp{1}; Noise_q{1}; Noise_the{1}; Noise_alt{1}; Noise_Ax{1}; Noise_Az{1}; u{2} ]';
            outputvar = '[ Perf_tas; Perf_alp; W_thro; W_elev; tas_c-LTI_lon(1)-N_tas; alt_c-LTI_lon(5)-N_alt; LTI_lon(2)+N_alp; LTI_lon(3)+N_q; LTI_lon(4)+N_theta; LTI_lon(6)+N_Ax; LTI_lon(7)+N_Az ]';
            
            input_to_LTI_lon = '[ u(1)+D_thro; u(2)+D_elev ]';
            
            input_to_Model_tas = '[ tas_c ]';
            input_to_Model_alt = '[ alt_c ]';
            
            input_to_Perf_tas = '[ Model_tas-LTI_lon(1) ]';
            input_to_Perf_alt = '[ Model_alt-LTI_lon(5) ]';
            
            input_to_D_thro = '[ Dist_thro ]';
            input_to_D_elev = '[ Dist_elev ]';
            
            input_to_W_thro = '[ u(1)+D_thro ]';
            input_to_W_elev = '[ u(2)+D_elev ]';
            
            input_to_N_tas = '[ Noise_tas ]';
            input_to_N_alp = '[ Noise_alp ]';
            input_to_N_q = '[ Noise_q ]';
            input_to_N_theta = '[ Noise_the ]';
            input_to_N_alt = '[ Noise_alt ]';
            input_to_N_Ax = '[ Noise_Ax ]';
            input_to_N_Az = '[ Noise_Az ]';
            
            sysoutname = 'LTI_G_lon';
            cleanupsysic = 'yes'; 
            sysic
            
            % Synthesis
            [K_lon_hinfs(:,:,i,j,k),~,gamma_lon_hinfs(i,j,k)] = hinfsyn(LTI_G_lon,nmeas_lon,ncont_lon);
            disp(['H-Infinity Synthesis DONE'])
            gamma_lon_hinfs
            K_lon_hinf = K_lon_hinfs(:,:,i,j,k);
            
            % Closed-loop interconnection
%             systemnames = ' LTI_lon Model_alp Perf_alp W_elev D_elev N_alp N_q K_lon_hinf ';
%             inputvar = '[ Pilot_lon{1}; Dist_elev{1}; Noise_alp{1}; Noise_q{1} ]';
%             outputvar = '[ Perf_alp; W_elev; LTI_lon ]';
%             input_to_LTI_lon = '[ K_lon_hinf+D_elev ]';
%             input_to_Model_alp = '[ Pilot_lon ]';
%             input_to_Perf_alp = '[ Model_alp-LTI_lon(1) ]';
%             input_to_W_elev = '[ K_lon_hinf+D_elev ]';
%             input_to_D_elev = '[ Dist_elev ]';
%             input_to_N_alp = '[ Noise_alp ]';
%             input_to_N_q = '[ Noise_q ]';
%             input_to_K_lon_hinf = '[ Pilot_lon-LTI_lon(1)-N_alp; LTI_lon(2)+N_q ]';
%             sysoutname = 'CL_lon_hinf';
%             cleanupsysic = 'yes'; 
%             sysic
% 
%             CL_lon_hinfs(:,:,i,j,k) = CL_lon_hinf;
        end
    end
end

%% Longitudinal H-infinity control design (augmented system)
% Weight function design
Model_tas = tf(16,[1,5.6,16])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
Model_alt = tf(16,[1,5.6,16])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
Perf_tas = tf([0.1,5.973],[1,0.176]);
Perf_alt = tf([0.1,5.973],[1,0.176]);
D_elev = 0.01;
W_elev = [tf([0.8,42.91],[1,119.2]),0;0,tf([0.8,42.91],[1,119.2])];
N_alp = 0.01;
N_q = 0.1;

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

%% Longitudinal H-infinity control design
% Weight function design
Model_alp = tf(16,[1,5.6,16])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
Perf_alp = tf([0.1,5.973],[1,0.176]);
D_elev = 0.01;
W_elev = tf([0.8,42.91],[1,119.2]);
N_alp = 0.01;
N_q = 0.1;

tic
for i = 1:num_tas
    for j = 1:num_alt
        for k = 1:num_mor
            trim_point = (i-1)*num_alt*num_mor + (j-1)*num_mor + k;
            
            LTI_lon = GTM_lon(:,:,i,j,k);
            
            % LTI control interconnection
            systemnames = ' LTI_lon Model_alp Perf_alp D_elev W_elev N_alp N_q ';
            inputvar = '[ Pilot_lon{1}; Dist_elev{1}; Noise_alp{1}; Noise_q{1}; u ]';
            outputvar = '[ Perf_alp; W_elev; Pilot_lon-LTI_lon(1)-N_alp; LTI_lon(2)+N_q ]';
            input_to_LTI_lon = '[ u+D_elev ]';
            input_to_Model_alp = '[ Pilot_lon ]';
            input_to_Perf_alp = '[ Model_alp-LTI_lon(1) ]';
            input_to_D_elev = '[ Dist_elev ]';
            input_to_W_elev = '[ u+D_elev ]';
            input_to_N_alp = '[ Noise_alp ]';
            input_to_N_q = '[ Noise_q ]';
            sysoutname = 'LTI_G_lon';
            cleanupsysic = 'yes'; 
            sysic
            
            % Synthesis
            [K_lon_hinfs(:,:,i,j,k),~,gamma_lon_hinfs(i,j,k)] = hinfsyn(LTI_G_lon,nmeas_lon,ncont_lon);
            disp(['H-Infinity Synthesis DONE'])
            gamma_lon_hinfs
            K_lon_hinf = K_lon_hinfs(:,:,i,j,k);
            
            % Closed-loop interconnection
            systemnames = ' LTI_lon Model_alp Perf_alp W_elev D_elev N_alp N_q K_lon_hinf ';
            inputvar = '[ Pilot_lon{1}; Dist_elev{1}; Noise_alp{1}; Noise_q{1} ]';
            outputvar = '[ Perf_alp; W_elev; LTI_lon ]';
            input_to_LTI_lon = '[ K_lon_hinf+D_elev ]';
            input_to_Model_alp = '[ Pilot_lon ]';
            input_to_Perf_alp = '[ Model_alp-LTI_lon(1) ]';
            input_to_W_elev = '[ K_lon_hinf+D_elev ]';
            input_to_D_elev = '[ Dist_elev ]';
            input_to_N_alp = '[ Noise_alp ]';
            input_to_N_q = '[ Noise_q ]';
            input_to_K_lon_hinf = '[ Pilot_lon-LTI_lon(1)-N_alp; LTI_lon(2)+N_q ]';
            sysoutname = 'CL_lon_hinf';
            cleanupsysic = 'yes'; 
            sysic

            CL_lon_hinfs(:,:,i,j,k) = CL_lon_hinf;
        end
    end
end
toc

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
            mag_elev_to_alpha_CL_lon_hinfs(:,trim_point) = mag2db(squeeze(mag_CL_lon_hinfs(3,1,:))).';
            phase_elev_to_alpha_CL_lon_hinfs(:,trim_point) = squeeze(phase_CL_lon_hinfs(3,1,:)).';
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
if rateBound == 1
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

%% Lateral H-infinity control design
% Weight function
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

for i = 1:num_tas
    for j = 1:num_alt
        for k = 1:num_mor
            LTI_lat = GTM_lat(:,:,i,j,k);
            
            % Control design interconnection
            systemnames = ' LTI_lat Model_bet Model_p Perf_bet Perf_p D_ail D_rud W_ail W_rud N_bet N_p N_r ';
            inputvar = '[ Pilot_lat{1}; Pilot_ped{1}; Dist_ail{1}; Dist_rud{1}; Noise_bet{1}; Noise_p{1}; Noise_r{1}; u{2} ]';
            outputvar = '[ Perf_bet; Perf_p; W_ail; W_rud; LTI_lat; Pilot_lat; Pilot_ped-LTI_lat(1)-N_bet; LTI_lat(2)+N_p; LTI_lat(3)+N_r ]';
            input_to_LTI_lat = '[ u(1)+D_ail; u(2)+D_rud ]';
            input_to_Model_bet = '[ Pilot_ped ]';
            input_to_Model_p = '[ Pilot_lat ]';
            input_to_Perf_bet = '[ Model_bet-LTI_lat(1) ]';
            input_to_Perf_p = '[ Model_p-LTI_lat(2) ]';
            input_to_D_ail = '[ Dist_ail ]';
            input_to_D_rud = '[ Dist_rud ]';
            input_to_W_ail = '[ u(1)+D_ail ]';
            input_to_W_rud = '[ u(2)+D_rud ]';
            input_to_N_bet = '[ Noise_bet ]';
            input_to_N_p = '[ Noise_p ]';
            input_to_N_r = '[ Noise_r ]';
            sysoutname = 'LTI_G_lat';
            cleanupsysic = 'yes'; 
            sysic

            % Controller synthesis
            [K_lat_hinfs(:,:,i,j,k),A,gamma_lat_hinfs(i,j,k)] = hinfsyn(LTI_G_lat,nmeas_lat,ncont_lat);
            disp(['H-Infinity Synthesis DONE'])
            gamma_lat_hinfs
            K_lat_hinf = K_lat_hinfs(:,:,i,j,k);
            
            % Closed-loop interconnection
            systemnames = ' LTI_lat Model_bet Model_p Perf_bet Perf_p D_ail D_rud W_ail W_rud N_bet N_p N_r K_lat_hinf ';
            inputvar = '[ Pilot_lat{1}; Pilot_ped{1}; Dist_ail{1}; Dist_rud{1}; Noise_bet{1}; Noise_p{1}; Noise_r{1} ]';
            outputvar = '[ LTI_lat ]';
            input_to_LTI_lat = '[ LTI_lat(1)+D_ail; LTI_lat(2)+D_rud ]';
            input_to_Model_bet = '[ Pilot_ped ]';
            input_to_Model_p = '[ Pilot_lat ]';
            input_to_Perf_bet = '[ Model_bet-LTI_lat(1) ]';
            input_to_Perf_p = '[ Model_p-LTI_lat(2) ]';
            input_to_D_ail = '[ Dist_ail ]';
            input_to_D_rud = '[ Dist_rud ]';
            input_to_W_ail = '[ LTI_lat(1)+D_ail ]';
            input_to_W_rud = '[ LTI_lat(2)+D_rud ]';
            input_to_N_bet = '[ Noise_bet ]';
            input_to_N_p = '[ Noise_p ]';
            input_to_N_r = '[ Noise_r ]';
            input_to_K_lat_hinf = '[ Pilot_lat; Pilot_ped-LTI_lat(1)-N_bet; LTI_lat(2)+N_p; LTI_lat(3)+N_r ]';
            sysoutname = 'CL_lat_hinf';
            cleanupsysic = 'yes'; 
            sysic

            CL_lat_hinfs(:,:,i,j,k) = CL_lat_hinf;
        end
    end
end

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
