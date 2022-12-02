% actuator model
P_a_eng = tf(ss([-1.336,-0.7314;1,0],[1;0],[-0.1474   0.7314],0)); % throttle lag model
P_a_ele = 2*pi*5/(s+2*pi*5); % elevator model
Pa = blkdiag(P_a_eng,P_a_ele); % actuator model

% reference scale
W_r_tas = 4*knots2fps; % tas reference scale [ft/s]
W_r_alt = 10; % altitude reference scale [ft]
Wr = blkdiag(W_r_tas,W_r_alt);

% ideal response
omega_tas = 2; % ideal tas model natural frequency [rad/s]
omega_alt = 2; % ideal alt model natural frequency [rad/s]
zeta_tas = .8; % ideal tas model damping ratio []
zeta_alt = .8; % ideal alt model damping ratio []
W_m_tas = tf(omega_tas^2,[1 2*zeta_tas*omega_tas omega_tas^2]); % ideal tas model (2nd-order system)
W_m_alt = tf(omega_alt^2,[1 2*zeta_alt*omega_alt omega_alt^2]); % ideal alt model (2nd-order system)
Wm = blkdiag(W_m_tas,W_m_alt); % ideal response

% sensor measurement noise
W_n_tas   = 0.06*knots2fps; % 0.06 kt tas white noise (ADS)
W_n_alp   = 0.031*d2r; % 0.031 deg alp white noise (ADS)
W_n_q     = 0.3857*d2r; % 0.3857 deg/s q white noise (MAG3)
W_n_theta = 0.02*d2r; % 0.02 deg theta white noise (MIDG II)
W_n_alt   = 0.01*m2ft; % altitude white noise, sensor fusion (barometer + MAG3 IMU) [ft]
Wn = blkdiag(W_n_tas,W_n_alp,W_n_q,W_n_theta,W_n_alt);

% control weight
W_c_eng = (s/0.5+1)/(s/120+1)*0.001; % throttle weight [%]
W_c_ele = (s/0.5+1)/(s/120+1)*0.001; % elevator weight [deg]
Wc = blkdiag(W_c_eng,W_c_ele);

% performance weight
W_p_tas = (s/120+1)/(s/0.005+1)/(0.06*knots2fps*2); % tas tracking error weight
W_p_alt = (s/120+1)/(s/0.005+1)/(0.01*m2ft*2); % alt tracking error weight
Wp = blkdiag(W_p_tas,W_p_alt);

% time delay
TD30ms = pade(exp(-30e-3*s),1);
TD70ms = pade(exp(-70e-3*s),1);
Td = blkdiag(TD30ms,TD30ms,TD30ms,TD70ms,TD70ms);

% actuator disturbance weight
D_eng = 1/(s/0.5+1)*0.01;
D_ele = 1/(s/0.5+1)*0.01;
Da = blkdiag(D_eng,D_ele);

% generalized plant
systemnames    = 'Plon5 Wm Wn Wc Wp Wr Pa Td Da';
inputvar       = '[r{2} ; n{5} ; d{2} ; u{2}]';
outputvar      = '[Wc ; Wp ; Wr(1)-Td(1) ; Wr(2)-Td(5) ; Td(2) ; Td(3) ; Td(4)]';
input_to_Plon5 = '[Pa+Da]';
input_to_Wm    = '[Wr]';
input_to_Wn    = '[n]';
input_to_Wc    = '[u]';
input_to_Wp    = '[Wm(1)-Plon5(1) ; Wm(2)-Plon5(5)]';
input_to_Wr    = '[r]';
input_to_Pa    = '[u]';
input_to_Td    = '[Plon5+Wn]';
input_to_Da    = '[d]';
sysoutname     = 'OLlon5';
cleanupsysic   = 'yes';
sysic

ncontlon5 = 2; % number of control
nmeaslon5 = 5; % number of measurement

% Hinf synthesis
clearvars Klon5hinf_grid gamlon5hinf_grid % clear old variables
tic
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor 
    OLlon5_pt = OLlon5.Data(:,:,i,j,k);
    [~,~,gamlon5_hinf_pt_opt]        = hinfsyn(OLlon5_pt,nmeaslon5,ncontlon5);
    [Klon5hinf_pt,~,gamlon5hinf_pt] = hinfsyn(OLlon5_pt,nmeaslon5,ncontlon5,1.2*gamlon5_hinf_pt_opt);
    fprintf('Lon5 Hinf #%d Gam = %.5g (%.3g%% back-off from %.5g)\n',sub2ind([num_mor num_alt num_sas],k,j,i),gamlon5hinf_pt,(gamlon5hinf_pt/gamlon5_hinf_pt_opt-1)*100,gamlon5_hinf_pt_opt)
    Klon5hinf_grid(:,:,i,j,k) = Klon5hinf_pt;
    gamlon5hinf_grid(i,j,k) = gamlon5hinf_pt;
end
end
end
timelon5hinf = toc;
fprintf('Lon5 Hinf Syn Time: %.1f sec \n',timelon5hinf)
Klon5hinf = pss(Klon5hinf_grid,domain);

% LPVu synthesis
tic
[Klon5lpvu,gamlon5lpvu,infolon5lpvu] = lpvsyn(OLlon5,nmeaslon5,ncontlon5);
timelon5lpvu = toc;
fprintf('Lon5 LPVu Syn Time: %.1f sec \n',timelon5lpvu)
fprintf('Lon5 LPVu Gam = %.5g (%.3g%% back-off from %.5g)\n',gamlon5lpvu,(gamlon5lpvu/infolon5lpvu.MinGamma-1)*100,infolon5lpvu.MinGamma)

% input & output scaling
Ps = diag([1/knots2fps r2d r2d r2d 1]); % [kt] x [deg] x [deg] x [deg] x [ft]
Ks = diag([1 1]); % [%] x [deg]

% loop closing with hinf
systemnames        = 'Plon5 Klon5hinf Wr Pa Td Ps Ks';
inputvar           = '[r{2}]';
outputvar          = '[Ps ; Ks]';
input_to_Plon5     = '[Pa]';
input_to_Klon5hinf = '[Wr(1)-Td(1) ; Wr(2)-Td(5) ; Td(2) ; Td(3) ; Td(4)]';
input_to_Wr        = '[r]';
input_to_Pa        = '[Klon5hinf]';
input_to_Td        = '[Plon5]';
input_to_Ps        = '[Plon5]';
input_to_Ks        = '[Klon5hinf]';
sysoutname         = 'CLlon5hinf';
cleanupsysic       = 'yes'; 
sysic

% loop closing with lpvu
systemnames        = 'Plon5 Klon5lpvu Wr Pa Td Ps Ks';
inputvar           = '[r{2}]';
outputvar          = '[Ps ; Ks]';
input_to_Plon5     = '[Pa]';
input_to_Klon5lpvu = '[Wr(1)-Td(1) ; Wr(2)-Td(5) ; Td(2) ; Td(3) ; Td(4)]';
input_to_Wr        = '[r]';
input_to_Pa        = '[Klon5lpvu]';
input_to_Td        = '[Plon5]';
input_to_Ps        = '[Plon5]';
input_to_Ks        = '[Klon5lpvu]';
sysoutname         = 'CLlon5lpvu';
cleanupsysic       = 'yes'; 
sysic

% collect weights
Wlon5.Wm = Wm;
Wlon5.Wn = Wn;
Wlon5.Wc = Wc;
Wlon5.Wp = Wp;
Wlon5.Wr = Wr;
Wlon5.Pa = Pa;
Wlon5.Td = Td;
Wlon5.Da = Da;
Wlon5.Ps = Ps;
Wlon5.Ks = Ks;
