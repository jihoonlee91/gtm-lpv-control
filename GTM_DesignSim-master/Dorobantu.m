%% Longitudinal H-infinity control design

A_sp = [-2.4714 0.9514;
    -43.9070 -3.4738];
B_sp = [-0.0043;-0.7662];
C_sp = eye(2);
D_sp = 0;
LTI_lon = ss(A_sp,B_sp,C_sp,D_sp);

% Weight function design
Model_alp = tf(16,[1,5.6,16])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
Perf_alp = tf([0.1,5.973],[1,0.176]);
D_elev = 0.01;
W_elev = 0.01;
N_alp = 0.01;
N_q = 0.1;


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
[K_lon_hinf,~,gamma_lon_hinf] = hinfsyn(LTI_G_lon,2,1);
gamma_lon_hinf

% Closed-loop interconnection
systemnames = ' LTI_lon K_lon_hinf ';
inputvar = '[ Pilot_lon{1} ]';
outputvar = '[ LTI_lon ]';
input_to_LTI_lon = '[ K_lon_hinf ]';
input_to_K_lon_hinf = '[ Pilot_lon-LTI_lon(1); LTI_lon(2) ]';
sysoutname = 'CL_lon_hinf';
cleanupsysic = 'yes'; 
sysic

bode(CL_lon_hinf)
step(CL_lon_hinf)

%%
A = [-0.0665 -11.4608 0.1439 -32.1740;
    -0.0035 -2.4714 0.9514 0;
    -0.0090 -43.9070 -3.4738 0;
    0 0 1 0];
B = [0.1419 -0.0435;
    -0.0001273 -0.0043;
    0.01917 -0.7662;
    0 0];
C = eye(4);
D = zeros(4,2);
LTI_lon = ss(A,B,C,D);

% Weight function design
Model_tas = tf(16,[1,5.6,16])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
Model_alp = tf(16,[1,5.6,16])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
Perf_tas = tf([0.1,5.973],[1,0.176]);
Perf_alp = tf([0.1,5.973],[1,0.176]);
D_thro = 0.01;
D_elev = 0.01;
W_thro = 0.01;
W_elev = 0.01;
N_tas = 0.01;
N_alp = 0.01;
N_q = 0.1;
N_the = 0.1;

% LTI control interconnection
systemnames = ' LTI_lon Model_tas Model_alp Perf_tas Perf_alp D_thro D_elev W_thro W_elev N_tas N_alp N_q N_the ';
inputvar = '[ tas_c{1}; alp_c{1}; Dist_thro{1} Dist_elev{1}; Noise_tas{1}; Noise_alp{1}; Noise_q{1}; Noise_the{1}; u{2} ]';
outputvar = '[ Perf_tas; Perf_alp; W_thro; W_elev; tas_c-LTI_lon(1)-N_tas; alp_c-LTI_lon(2)-N_alp; LTI_lon(3)+N_q; LTI_lon(4)+N_the ]';
input_to_LTI_lon = '[ u(1)+D_thro; u(2)+D_elev ]';
input_to_Model_tas = '[ tas_c ]';
input_to_Model_alp = '[ alp_c ]';
input_to_Perf_tas = '[ Model_tas-LTI_lon(1) ]';
input_to_Perf_alp = '[ Model_alp-LTI_lon(2) ]';
input_to_D_thro = '[ Dist_thro ]';
input_to_D_elev = '[ Dist_elev ]';
input_to_W_thro = '[ u(1)+D_thro ]';
input_to_W_elev = '[ u(2)+D_elev ]';
input_to_N_tas = '[ Noise_tas ]';
input_to_N_alp = '[ Noise_alp ]';
input_to_N_q = '[ Noise_q ]';
input_to_N_the = '[ Noise_the ]';
sysoutname = 'LTI_G_lon';
cleanupsysic = 'yes'; 
sysic

% Synthesis
[K_lon_hinf,~,gamma_lon_hinf] = hinfsyn(LTI_G_lon,4,2);
gamma_lon_hinf

% Closed-loop interconnection
% systemnames = ' LTI_lon K_lon_hinf ';
% inputvar = '[ Pilot_lon{1} ]';
% outputvar = '[ LTI_lon ]';
% input_to_LTI_lon = '[ K_lon_hinf ]';
% input_to_K_lon_hinf = '[ Pilot_lon-LTI_lon(1); LTI_lon(2) ]';
% sysoutname = 'CL_lon_hinf';
% cleanupsysic = 'yes'; 
% sysic
% 
% bode(CL_lon_hinf)
% step(CL_lon_hinf)
