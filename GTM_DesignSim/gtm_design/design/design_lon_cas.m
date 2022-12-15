% actuator model
P_a_ele = 2*pi*5/(s+2*pi*5); % elevator model
Pa = blkdiag(P_a_ele); % actuator model

% reference scale (maximum expected command)
W_r_an  = 3*g; % an reference scale [ft/s^2]
Wr = blkdiag(W_r_an);

% ideal response
omega_an = 2.5;
zeta_an = .8;
W_m_an  = omega_an^2/(s^2+2*zeta_an*omega_an*s+omega_an^2); % ideal an model (1st-order system)
Wm = blkdiag(W_m_an); % ideal response

% sensor measurement noise
W_n_alp = 0.031*d2r; % 0.031 deg alp white noise (ADS)
W_n_q   = 0.3857*d2r; % 0.3857 deg/s q white noise (MAG3)
W_n_an  = 0.0082*g; % 0.0082g white noise (MAG3)
Wn = blkdiag(W_n_alp,W_n_q,W_n_an);

% control weight
W_c_ele = (s/0.5+1)/(s/120+1)*0.0001; % elevator weight [deg]
Wc = blkdiag(W_c_ele);

% performance weight
W_p_an = (s/omega_an+1)/(s/0.04+1)/(0.0082*g*5); % an tracking error weight
Wp = blkdiag(W_p_an);

% time delay
TD30ms = pade(exp(-30e-3*s),1);
Td = blkdiag(TD30ms,TD30ms,TD30ms);

% actuator disturbance weight
D_ele = 1/(s/0.5+1)*0.01;
Da = blkdiag(D_ele);

% generalized plant
systemnames    = 'Plon2 Wm Wn Wc Wp Wr Pa Td Da';
inputvar       = '[r{1} ; n{3} ; d{1} ; u{1}]';
outputvar      = '[Wc ; Wp ; Wr(1)+Td(3) ; Td(1) ; Td(2)]';
input_to_Plon2 = '[Pa+Da]';
input_to_Wm    = '[Wr]';
input_to_Wn    = '[n]';
input_to_Wc    = '[u]';
input_to_Wp    = '[Wm(1)+Plon2(3)]';
input_to_Wr    = '[r]';
input_to_Pa    = '[u]';
input_to_Td    = '[Plon2+Wn]';
input_to_Da    = '[d]';
sysoutname     = 'OLlon2';
cleanupsysic   = 'yes';
sysic

ncontlon2 = 1; % number of control
nmeaslon2 = 3; % number of measurement

% Hinf synthesis
clearvars Klon2hinf_grid gamlon2hinf_grid % clear old variables
tic
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor 
    OLlon2_pt = OLlon2.Data(:,:,i,j,k);
    [~,~,gamlon2_hinf_pt_opt]        = hinfsyn(OLlon2_pt,nmeaslon2,ncontlon2);
    [Klon2hinf_pt,~,gamlon2hinf_pt] = hinfsyn(OLlon2_pt,nmeaslon2,ncontlon2,1.2*gamlon2_hinf_pt_opt);
    fprintf('lon2 Hinf #%d Gam = %.5g (%.3g%% back-off from %.5g)\n',sub2ind([num_mor num_alt num_sas],k,j,i),gamlon2hinf_pt,(gamlon2hinf_pt/gamlon2_hinf_pt_opt-1)*100,gamlon2_hinf_pt_opt)
    Klon2hinf_grid(:,:,i,j,k) = Klon2hinf_pt;
    gamlon2hinf_grid(i,j,k) = gamlon2hinf_pt;
end
end
end
timelon2hinf = toc;
fprintf('lon2 Hinf Syn Time: %.1f sec \n',timelon2hinf)
Klon2hinf = pss(Klon2hinf_grid,domain);

% H2op synthesis
clearvars Klon2h2op_grid gamlon2h2op_grid % clear old variables
tic
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor 
    OLlon2_pt = OLlon2.Data(:,:,i,j,k);
    [Klon2h2op_pt,~,gamlon2h2op_pt] = h2syn(OLlon2_pt,nmeaslon2,ncontlon2);
    fprintf('lon2 H2op #%d Gam = %.5g\n',sub2ind([num_mor num_alt num_sas],k,j,i),gamlon2h2op_pt)
    Klon2h2op_grid(:,:,i,j,k) = Klon2h2op_pt;
    gamlon2h2op_grid(i,j,k) = gamlon2h2op_pt;
end
end
end
timelon2h2op = toc;
fprintf('lon2 H2op Syn Time: %.1f sec \n',timelon2h2op)
Klon2h2op = pss(Klon2h2op_grid,domain);

% LPVu synthesis
tic
[Klon2lpvu,gamlon2lpvu,infolon2lpvu] = lpvsyn(OLlon2,nmeaslon2,ncontlon2);
timelon2lpvu = toc;
fprintf('lon2 LPVu Syn Time: %.1f sec \n',timelon2lpvu)
fprintf('lon2 LPVu Gam = %.5g (%.3g%% back-off from %.5g)\n',gamlon2lpvu,(gamlon2lpvu/infolon2lpvu.MinGamma-1)*100,infolon2lpvu.MinGamma)

% input & output scaling
Ps = diag([r2d r2d -1/g]); % [deg] x [deg] x [g]
Ks = diag([1]); % [deg]

% loop closing with hinf
systemnames        = 'Plon2 Klon2hinf Wr Pa Td Ps Ks';
inputvar           = '[r{1}]';
outputvar          = '[Ps ; Ks]';
input_to_Plon2     = '[Pa]';
input_to_Klon2hinf = '[Wr(1)+Td(3) ; Td(1) ; Td(2)]';
input_to_Wr        = '[r]';
input_to_Pa        = '[Klon2hinf]';
input_to_Td        = '[Plon2]';
input_to_Ps        = '[Plon2]';
input_to_Ks        = '[Klon2hinf]';
sysoutname         = 'CLlon2hinf';
cleanupsysic       = 'yes'; 
sysic

% loop closing with h2op
systemnames        = 'Plon2 Klon2h2op Wr Pa Td Ps Ks';
inputvar           = '[r{1}]';
outputvar          = '[Ps ; Ks]';
input_to_Plon2     = '[Pa]';
input_to_Klon2h2op = '[Wr(1)+Td(3) ; Td(1) ; Td(2)]';
input_to_Wr        = '[r]';
input_to_Pa        = '[Klon2h2op]';
input_to_Td        = '[Plon2]';
input_to_Ps        = '[Plon2]';
input_to_Ks        = '[Klon2h2op]';
sysoutname         = 'CLlon2h2op';
cleanupsysic       = 'yes'; 
sysic

% loop closing with lpvu
systemnames        = 'Plon2 Klon2lpvu Wr Pa Td Ps Ks';
inputvar           = '[r{1}]';
outputvar          = '[Ps ; Ks]';
input_to_Plon2     = '[Pa]';
input_to_Klon2lpvu = '[Wr(1)+Td(3) ; Td(1) ; Td(2)]';
input_to_Wr        = '[r]';
input_to_Pa        = '[Klon2lpvu]';
input_to_Td        = '[Plon2]';
input_to_Ps        = '[Plon2]';
input_to_Ks        = '[Klon2lpvu]';
sysoutname         = 'CLlon2lpvu';
cleanupsysic       = 'yes'; 
sysic

% collect weights
Wlon2.Wm = Wm;
Wlon2.Wn = Wn;
Wlon2.Wc = Wc;
Wlon2.Wp = Wp;
Wlon2.Wr = Wr;
Wlon2.Pa = Pa;
Wlon2.Td = Td;
Wlon2.Da = Da;
Wlon2.Ps = Ps;
Wlon2.Ks = Ks;
