% actuator model
P_a_ail = 2*pi*5/(s+2*pi*5); % Bandwidth 5 Hz and 5+10 ms time delay
P_a_rud = 2*pi*5/(s+2*pi*5); % Bandwidth 5 Hz and 5+10 ms time delay
P_a_mor = 2*pi*0.5/(s+2*pi*0.5); % Bandwidth 5 Hz and 5+10 ms time delay
Pa = blkdiag(P_a_ail,P_a_rud,P_a_mor);

% maximum expected command
W_r_bet = 3*d2r;
W_r_p   = 10*d2r;
Wr = blkdiag(W_r_bet,W_r_p);

% ideal model
omega_bet = 2; % [rad/s]
zeta_bet = .8; % []
W_m_bet = tf(omega_bet^2,[1 2*zeta_bet*omega_bet omega_bet^2]);
omega_p = 10; % [rad/s]
zeta_p = .8; % []
W_m_p = tf(omega_p^2,[1 2*zeta_p*omega_p omega_p^2]);
Wm = blkdiag(W_m_bet,W_m_p);

% noise model
W_n_bet = 0.033*d2r; % 0.033 deg
W_n_p   = 0.5220*d2r; % 0.5220 deg/sec
W_n_r   = 0.3330*d2r; % 0.3330 deg/sec
Wn = blkdiag(W_n_bet,W_n_p,W_n_r);

% control weight
W_c_ail = (s/.1+1)/(s/120+1)*0.0001; % [deg]
W_c_rud = (s/.1+1)/(s/120+1)*0.0005; % [deg]
W_c_mor = (s/.1+1)/(s/120+1)*0.01; % [deg]
Wc = blkdiag(W_c_ail,W_c_rud,W_c_mor);

% performance weight
W_p_bet = (s/120+1)/(s/0.1+1)/(W_n_bet*2);
W_p_p = (s/120+1)/(s/0.1+1)/(W_n_p*2);
Wp = blkdiag(W_p_bet,W_p_p);

% time delay
TD30ms = pade(exp(-30e-3*s),1);
Td = blkdiag(TD30ms,TD30ms,TD30ms);

% actuator disturbance weight
D_ail = 1/(s/0.5+1)*0.01;
D_rud = 1/(s/0.5+1)*0.01;
D_mor = 1/(s/0.5+1)*0.01;
Da = blkdiag(D_ail,D_rud,D_mor);

% generalized plant
systemnames    = 'Plat3 Wm Wn Wc Wp Wr Pa Td Da';
inputvar       = '[r{2} ; n{3} ; d{3} ; u{3}]';
outputvar      = '[Wc ; Wp ; Wr(1)-Td(1) ; Wr(2)-Td(2) ; Td(3)]';
input_to_Plat3 = '[Pa+Da]';
input_to_Wm    = '[Wr]';
input_to_Wn    = '[n]';
input_to_Wc    = '[u]';
input_to_Wp    = '[Wm(1)-Plat3(1) ; Wm(2)-Plat3(2)]';
input_to_Wr    = '[r]';
input_to_Pa    = '[u]';
input_to_Td    = '[Plat3+Wn]';
input_to_Da    = '[d]';
sysoutname     = 'OLlat3';
cleanupsysic   = 'yes';
sysic

ncontlat3 = 3; % number of control
nmeaslat3 = 3; % number of measurement

% Hinf synthesis
clearvars Klat3hinf_grid gamlat3hinf_grid % clear old variables
tic
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor 
    OLlat3_pt = OLlat3.Data(:,:,i,j,k);
    [~,~,gamlat3_hinf_pt_opt]        = hinfsyn(OLlat3_pt,nmeaslat3,ncontlat3);
    [Klat3hinf_pt,~,gamlat3hinf_pt] = hinfsyn(OLlat3_pt,nmeaslat3,ncontlat3,1.2*gamlat3_hinf_pt_opt);
    fprintf('lat3 Hinf #%d Gam = %.5g (%.3g%% back-off from %.5g)\n',sub2ind([num_mor num_alt num_sas],k,j,i),gamlat3hinf_pt,(gamlat3hinf_pt/gamlat3_hinf_pt_opt-1)*100,gamlat3_hinf_pt_opt)
    Klat3hinf_grid(:,:,i,j,k) = Klat3hinf_pt;
    gamlat3hinf_grid(i,j,k) = gamlat3hinf_pt;
end
end
end
timelat3hinf = toc;
fprintf('lat3 Hinf Syn Time: %.1f sec \n',timelat3hinf)
Klat3hinf = pss(Klat3hinf_grid,domain);

% H2op synthesis
clearvars Klat3h2op_grid gamlat3h2op_grid % clear old variables
tic
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor 
    OLlat3_pt = OLlat3.Data(:,:,i,j,k);
    [Klat3h2op_pt,~,gamlat3h2op_pt] = h2syn(OLlat3_pt,nmeaslat3,ncontlat3);
    fprintf('lat3 H2op #%d Gam = %.5g\n',sub2ind([num_mor num_alt num_sas],k,j,i),gamlat3h2op_pt)
    Klat3h2op_grid(:,:,i,j,k) = Klat3h2op_pt;
    gamlat3h2op_grid(i,j,k) = gamlat3h2op_pt;
end
end
end
timelat3h2op = toc;
fprintf('lat3 H2op Syn Time: %.1f sec \n',timelat3h2op)
Klat3h2op = pss(Klat3h2op_grid,domain);

% LPVu synthesis
tic
[Klat3lpvu,gamlat3lpvu,infolat3lpvu] = lpvsyn(OLlat3,nmeaslat3,ncontlat3);
timelat3lpvu = toc;
fprintf('lat3 LPVu Syn Time: %.1f sec \n',timelat3lpvu)
fprintf('lat3 LPVu Gam = %.5g (%.3g%% back-off from %.5g)\n',gamlat3lpvu,(gamlat3lpvu/infolat3lpvu.MinGamma-1)*100,infolat3lpvu.MinGamma)

% input & output scaling
Ps = diag([r2d r2d r2d]); % [deg] x [deg] x [deg] x [deg]
Ks = diag([1 1 1]); % [deg] x [deg] x [%]

% loop closing with hinf
systemnames        = 'Plat3 Klat3hinf Wr Pa Td Ps Ks';
inputvar           = '[r{2}]';
outputvar          = '[Ps ; Ks]';
input_to_Plat3     = '[Pa]';
input_to_Klat3hinf = '[Wr(1)-Td(1) ; Wr(2)-Td(2) ; Td(3)]';
input_to_Wr        = '[r]';
input_to_Pa        = '[Klat3hinf]';
input_to_Td        = '[Plat3]';
input_to_Ps        = '[Plat3]';
input_to_Ks        = '[Klat3hinf]';
sysoutname         = 'CLlat3hinf';
cleanupsysic       = 'yes'; 
sysic

% loop closing with h2op
systemnames        = 'Plat3 Klat3h2op Wr Pa Td Ps Ks';
inputvar           = '[r{2}]';
outputvar          = '[Ps ; Ks]';
input_to_Plat3     = '[Pa]';
input_to_Klat3h2op = '[Wr(1)-Td(1) ; Wr(2)-Td(2) ; Td(3)]';
input_to_Wr        = '[r]';
input_to_Pa        = '[Klat3h2op]';
input_to_Td        = '[Plat3]';
input_to_Ps        = '[Plat3]';
input_to_Ks        = '[Klat3h2op]';
sysoutname         = 'CLlat3h2op';
cleanupsysic       = 'yes'; 
sysic

% loop closing with lpvu
systemnames        = 'Plat3 Klat3lpvu Wr Pa Td Ps Ks';
inputvar           = '[r{2}]';
outputvar          = '[Ps ; Ks]';
input_to_Plat3     = '[Pa]';
input_to_Klat3lpvu = '[Wr(1)-Td(1) ; Wr(2)-Td(2) ; Td(3)]';
input_to_Wr        = '[r]';
input_to_Pa        = '[Klat3lpvu]';
input_to_Td        = '[Plat3]';
input_to_Ps        = '[Plat3]';
input_to_Ks        = '[Klat3lpvu]';
sysoutname         = 'CLlat3lpvu';
cleanupsysic       = 'yes'; 
sysic

% collect weights
Wlat3.Wm = Wm;
Wlat3.Wn = Wn;
Wlat3.Wc = Wc;
Wlat3.Wp = Wp;
Wlat3.Wr = Wr;
Wlat3.Pa = Pa;
Wlat3.Td = Td;
Wlat3.Da = Da;
Wlat3.Ps = Ps;
Wlat3.Ks = Ks;
