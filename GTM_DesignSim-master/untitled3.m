%% Longitudinal H-infinity control design

A_sp = [-0.0665 -11.4608 0.1439 -32.1740;
    -0.0035 -2.4714 0.9514 0;
    -0.0090 -43.9070 -3.4738 0;
    0 0 1 0];
B_sp = [-0.0435;-0.0043;-0.7662;0];
C_sp = eye(2);
D_sp = zeros(2,1);
LTI_lon = ss(A(2:3,2:3),B(2:3,:),C,D);

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
