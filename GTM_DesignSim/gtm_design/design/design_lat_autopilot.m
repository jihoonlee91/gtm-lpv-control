% actuator model
P_a_ail = 2*pi*5/(s+2*pi*5); % Bandwidth 5 Hz and 5+10 ms time delay
P_a_rud = 2*pi*5/(s+2*pi*5); % Bandwidth 5 Hz and 5+10 ms time delay
P_a_mor = 2*pi*0.5/(s+2*pi*0.5); % Bandwidth 5 Hz and 5+10 ms time delay
Pa = blkdiag(P_a_ail,P_a_rud,P_a_mor);

% maximum expected command
W_r_bet = 3*d2r;
W_r_phi = 30*d2r;
Wr = blkdiag(W_r_bet,W_r_phi);

% ideal model
omega_bet = 2; % [rad/s]
zeta_bet = .8; % []
W_m_bet = tf(omega_bet^2,[1 2*zeta_bet*omega_bet omega_bet^2]);
omega_phi = 5; % [rad/s]
zeta_phi = .8; % []
W_m_phi = tf(omega_phi^2,[1 2*zeta_phi*omega_phi omega_phi^2]);
Wm = blkdiag(W_m_bet,W_m_phi);

% noise model
W_n_bet = 0.033*d2r; % 0.033 deg
W_n_p   = 0.5220*d2r; % 0.5220 deg/sec
W_n_r   = 0.3330*d2r; % 0.3330 deg/sec
W_n_phi = 0.02*d2r; % 0.02 deg
Wn = blkdiag(W_n_bet,W_n_p,W_n_r,W_n_phi);

% control weight
W_c_ail = (s/.1+1)/(s/120+1)*0.0001; % [deg]
W_c_rud = (s/.1+1)/(s/120+1)*0.0005; % [deg]
W_c_mor = (s/.1+1)/(s/120+1)*0.01; % [deg]
Wc = blkdiag(W_c_ail,W_c_rud,W_c_mor);

% performance weight
W_p_bet = (s/120+1)/(s/0.1+1)/(W_n_bet*2);
W_p_phi = (s/120+1)/(s/0.003+1)/(W_n_phi*2);
Wp = blkdiag(W_p_bet,W_p_phi);

% time delay
TD70ms = pade(exp(-70e-3*s),1);
Td = blkdiag(TD30ms,TD30ms,TD30ms,TD70ms);

% actuator disturbance weight
D_ail = 1/(s/0.5+1)*0.01;
D_rud = 1/(s/0.5+1)*0.01;
D_mor = 1/(s/0.5+1)*0.01;
Da = blkdiag(D_ail,D_rud,D_mor);

% generalized plant
systemnames    = 'Plat4 Wm Wn Wc Wp Wr Pa Td Da';
inputvar       = '[r{2} ; n{4} ; d{3} ; u{3}]';
outputvar      = '[Wc ; Wp ; Wr(1)-Td(1) ; Wr(2)-Td(4) ; Td(2) ; Td(3)]';
input_to_Plat4 = '[Pa+Da]';
input_to_Wm    = '[Wr]';
input_to_Wn    = '[n]';
input_to_Wc    = '[u]';
input_to_Wp    = '[Wm(1)-Plat4(1) ; Wm(2)-Plat4(4)]';
input_to_Wr    = '[r]';
input_to_Pa    = '[u]';
input_to_Td    = '[Plat4+Wn]';
input_to_Da    = '[d]';
sysoutname     = 'OLlat4';
cleanupsysic   = 'yes';
sysic

ncontlat4 = 3; % number of control
nmeaslat4 = 4; % number of measurement

% Hinf synthesis
clearvars Klat4hinf_grid gamlat4hinf_grid % clear old variables
tic
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor 
    OLlat4_pt = OLlat4.Data(:,:,i,j,k);
    [~,~,gamlat4_hinf_pt_opt]        = hinfsyn(OLlat4_pt,nmeaslat4,ncontlat4);
    [Klat4hinf_pt,~,gamlat4hinf_pt] = hinfsyn(OLlat4_pt,nmeaslat4,ncontlat4,1.2*gamlat4_hinf_pt_opt);
    fprintf('lat4 Hinf #%d Gam = %.5g (%.3g%% back-off from %.5g)\n',sub2ind([num_mor num_alt num_sas],k,j,i),gamlat4hinf_pt,(gamlat4hinf_pt/gamlat4_hinf_pt_opt-1)*100,gamlat4_hinf_pt_opt)
    Klat4hinf_grid(:,:,i,j,k) = Klat4hinf_pt;
    gamlat4hinf_grid(i,j,k) = gamlat4hinf_pt;
end
end
end
timelat4hinf = toc;
fprintf('lat4 Hinf Syn Time: %.1f sec \n',timelat4hinf)
Klat4hinf = pss(Klat4hinf_grid,domain);

% H2op synthesis
clearvars Klat4h2op_grid gamlat4h2op_grid % clear old variables
tic
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor 
    OLlat4_pt = OLlat4.Data(:,:,i,j,k);
    [Klat4h2op_pt,~,gamlat4h2op_pt] = h2syn(OLlat4_pt,nmeaslat4,ncontlat4);
    fprintf('lat4 H2op #%d Gam = %.5g\n',sub2ind([num_mor num_alt num_sas],k,j,i),gamlat4h2op_pt)
    Klat4h2op_grid(:,:,i,j,k) = Klat4h2op_pt;
    gamlat4h2op_grid(i,j,k) = gamlat4h2op_pt;
end
end
end
timelat4h2op = toc;
fprintf('lat4 H2op Syn Time: %.1f sec \n',timelat4h2op)
Klat4h2op = pss(Klat4h2op_grid,domain);

% LPVu synthesis
tic
[Klat4lpvu,gamlat4lpvu,infolat4lpvu] = lpvsyn(OLlat4,nmeaslat4,ncontlat4);
timelat4lpvu = toc;
fprintf('lat4 LPVu Syn Time: %.1f sec \n',timelat4lpvu)
fprintf('lat4 LPVu Gam = %.5g (%.3g%% back-off from %.5g)\n',gamlat4lpvu,(gamlat4lpvu/infolat4lpvu.MinGamma-1)*100,infolat4lpvu.MinGamma)

% input & output scaling
Ps = diag([r2d r2d r2d r2d]); % [deg] x [deg] x [deg] x [deg]
Ks = diag([1 1 1]); % [deg] x [deg] x [%]

% loop closing with hinf
systemnames        = 'Plat4 Klat4hinf Wr Pa Td Ps Ks';
inputvar           = '[r{2}]';
outputvar          = '[Ps ; Ks]';
input_to_Plat4     = '[Pa]';
input_to_Klat4hinf = '[Wr(1)-Td(1) ; Wr(2)-Td(4) ; Td(2) ; Td(3)]';
input_to_Wr        = '[r]';
input_to_Pa        = '[Klat4hinf]';
input_to_Td        = '[Plat4]';
input_to_Ps        = '[Plat4]';
input_to_Ks        = '[Klat4hinf]';
sysoutname         = 'CLlat4hinf';
cleanupsysic       = 'yes'; 
sysic

% loop closing with h2op
systemnames        = 'Plat4 Klat4h2op Wr Pa Td Ps Ks';
inputvar           = '[r{2}]';
outputvar          = '[Ps ; Ks]';
input_to_Plat4     = '[Pa]';
input_to_Klat4h2op = '[Wr(1)-Td(1) ; Wr(2)-Td(4) ; Td(2) ; Td(3)]';
input_to_Wr        = '[r]';
input_to_Pa        = '[Klat4h2op]';
input_to_Td        = '[Plat4]';
input_to_Ps        = '[Plat4]';
input_to_Ks        = '[Klat4h2op]';
sysoutname         = 'CLlat4h2op';
cleanupsysic       = 'yes'; 
sysic

% loop closing with lpvu
systemnames        = 'Plat4 Klat4lpvu Wr Pa Td Ps Ks';
inputvar           = '[r{2}]';
outputvar          = '[Ps ; Ks]';
input_to_Plat4     = '[Pa]';
input_to_Klat4lpvu = '[Wr(1)-Td(1) ; Wr(2)-Td(4) ; Td(2) ; Td(3)]';
input_to_Wr        = '[r]';
input_to_Pa        = '[Klat4lpvu]';
input_to_Td        = '[Plat4]';
input_to_Ps        = '[Plat4]';
input_to_Ks        = '[Klat4lpvu]';
sysoutname         = 'CLlat4lpvu';
cleanupsysic       = 'yes'; 
sysic

% collect weights
Wlat4.Wm = Wm;
Wlat4.Wn = Wn;
Wlat4.Wc = Wc;
Wlat4.Wp = Wp;
Wlat4.Wr = Wr;
Wlat4.Pa = Pa;
Wlat4.Td = Td;
Wlat4.Da = Da;
Wlat4.Ps = Ps;
Wlat4.Ks = Ks;
