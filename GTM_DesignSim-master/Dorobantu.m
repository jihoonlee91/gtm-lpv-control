%% Longitudinal H-infinity control design

A_sp = [-2.4714 0.9514;
    -43.9070 -3.4738];
B_sp = [-0.0043;-0.7662]*180/pi;
C_sp = eye(2);
D_sp = 0;
LTI_lon = ss(A_sp,B_sp,C_sp,D_sp);

A_aug = [A_sp,B_sp;
    0,0,-5];
B_aug = [0;
    0;
    5];
C_aug = [1,0,0;
    0,1,0];
D_aug = zeros(2,1);
LTI_lon = ss(A_aug,B_aug,C_aug,D_aug);

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

%% Lateral H-infinity control design
A_lat = [-0.5229 0.0861 -0.9852 0.2347;
    -92.6425 -6.2736 2.0861 0;
    29.1843 -0.4833 -1.4043 0;
    0 1 0.0857 0];
B_lat = [-0.0002 0.0031;
    -0.9174 0.2321;
    -0.0523 -0.4436;
    0 0];
B_lat(:,1) = B_lat(:,1)*100;
B_lat(:,2) = B_lat(:,2)*180/pi;
C_lat = [eye(3) zeros(3,1)];
D_lat = zeros(3,2);
LTI_lat = ss(A_lat,B_lat,C_lat,D_lat);

set(LTI_lat,'Statename',{'beta','p','r','phi'},...
               'Inputname',{'dela','delr'},...
               'Outputname',{'beta','p','r'})

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
[K_lat_hinf,~,gamma_lat_hinf] = hinfsyn(LTI_G_lat,nmeas_lat,ncont_lat);
gamma_lat_hinf

% Closed-loop interconnection
systemnames = ' LTI_lat K_lat_hinf ';
inputvar = '[ Pilot_lat{1}; Pilot_ped{1} ]';
outputvar = '[ LTI_lat ]';
input_to_LTI_lat = '[ K_lat_hinf(1); K_lat_hinf(2) ]';
input_to_K_lat_hinf = '[ Pilot_lat; Pilot_ped-LTI_lat(1); LTI_lat(2); LTI_lat(3) ]';
sysoutname = 'CL_lat_hinf';
cleanupsysic = 'yes'; 
sysic

step(CL_lat_hinf)

%% Lateral H-infinity control design
A_lat = [-0.5229 0.0861 -0.9852 0.2347;
    -92.6425 -6.2736 2.0861 0;
    29.1843 -0.4833 -1.4043 0;
    0 1 0.0857 0];
B_lat = [-0.0002 0.0031;
    -0.9174 0.2321;
    -0.0523 -0.4436;
    0 0]*180/pi;
C_lat = [eye(3) zeros(3,1)];
D_lat = zeros(3,2);
LTI_lat = ss(A_lat,B_lat,C_lat,D_lat);
set(LTI_lat,'Statename',{'beta','p','r','phi'},...
               'Inputname',{'dela','delr'},...
               'Outputname',{'beta','p','r'})

Model_bet = tf(7.78,[1,3.92,7.78])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
Model_p = tf(-52.56,[1,0.04])*tf([1,-0.02],[1,6.47])*tf([1,7.5,34.67],[1,8.75,39.06])*tf(31.42,[1,31.42])*tf([-1,64.52],[1,64.52]);
Perf_bet = tf([0.1,5.973],[1,0.176]);
Perf_p = tf([0.1,5.973],[1,0.299]);
D_ail = 0.01;
D_rud = 0.01;
W_ail = tf([0.8,42.91],[1,119.2]);
W_rud = tf([0.8,42.91],[1,119.2]);
N_bet = 0.01;
N_p = 0.01;
N_r = 0.01;
        
% Control design interconnection
systemnames = ' LTI_lat Model_bet Model_p Perf_bet Perf_p D_ail D_rud W_ail W_rud N_bet N_p N_r ';
inputvar = '[ Pilot_lat{1}; Pilot_ped{1}; Dist_ail{1}; Dist_rud{1}; Noise_bet{1}; Noise_p{1}; Noise_r{1}; u{2} ]';
outputvar = '[ Perf_bet; Perf_p; W_ail; W_rud; Pilot_lat; Pilot_ped-LTI_lat(1)-N_bet; LTI_lat(2)+N_p; LTI_lat(3)+N_r ]';
% outputvar = '[ Perf_bet; Perf_p; W_ail; W_rud; Pilot_lat-LTI_lat(2)-N_bet; Pilot_ped-LTI_lat(1)-N_bet; LTI_lat(2)+N_p; LTI_lat(3)+N_r ]';
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
nmeas_lat = 4;
ncont_lat = 2;
[K_lat_hinf,~,gamma_lat_hinf] = hinfsyn(LTI_G_lat,nmeas_lat,ncont_lat);
gamma_lat_hinf

% Closed-loop interconnection
systemnames = ' LTI_lat K_lat_hinf ';
inputvar = '[ Pilot_lat{1}; Pilot_ped{1} ]';
outputvar = '[ LTI_lat ]';
input_to_LTI_lat = '[ K_lat_hinf(1); K_lat_hinf(2) ]';
input_to_K_lat_hinf = '[ Pilot_lat; Pilot_ped-LTI_lat(1); LTI_lat(2); LTI_lat(3) ]';
sysoutname = 'CL_lat_hinf';
cleanupsysic = 'yes'; 
sysic

[y,t] = step(CL_lat_hinf,10);
[yp,tp] = step(Model_p,10);
[yb,tb] = step(Model_bet,10);
% figure
subplot(1,2,1), plot(tp,yp)
hold on, plot(t,y(:,2,1)), hold off
grid on
legend('Model_p','Closed-loop')
subplot(1,2,2), plot(tb,yb)
hold on, plot(t,y(:,1,2)), hold off
grid on
legend('Model_bet','Closed-loop')