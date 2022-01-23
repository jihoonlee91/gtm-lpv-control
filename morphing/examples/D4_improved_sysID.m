%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                      Advanced sysID with LPVcore
%                        Demo 4: Improved sysID
%                          written by R. Toth
%                         version (10-07-2021)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clearvars; close all; clc;
rng(42);   % set random number generator seed so that results are reproducible
 
if ~exist('pmatrix','class'); cd(['..',filesep]); end  % Toolbox is not added to Matlab search path. Go to root of the toolbox
 

%% Define The Data Generating System
p= preal('p','dt','Range',[-1,1]);
a1=1-0.5*p-0.1*p^2;
a2=0.5-0.7*p-0.1*p^2;
b0=0.5-0.4*p+0.01*p^2;
b1=0.2-0.3*p-0.02*p^2;
c1=0.1*p;
d1=-1;
d2=0.2+0.1*p;
Ts=1;

sys = lpvidpoly([],{b0,b1},{1,c1},{1 d1 d2},{1, a1, a2}); % Create and idpolyobject => ARX model
sys.Ts = Ts;                   % Sampling time
sys.InputName = {'Input'};   % Naming of the signals
sys.OutputName = {'Output'};
sys.NoiseVariance = 0.0007;    % Specify noise variance

%% Data generation
N=4000;

% Estimation data
ue=1-2*rand(N,1);
pe=(0.5*sin(0.35*pi*(0:N-1))+0.5)';

[ye,~,~,yp] = lsim(sys,pe,ue);       % Generate data including noise signal
                                  % y - noisy output
                                  % yp - noise free output 
                                  
data_est = lpviddata(ye,pe,ue,Ts);   % Create an LPV data object
plot(data_est);                   % Show data

fprintf('The SNR is %f dB.\n', LPVcore.snr(yp,ye-yp));
 
% Create a noisy and a noise free validation data set
uv=idinput(N,'rgs');
pv=(0.5*sin(0.15*pi*(0:N-1)+0.25)+0.5)';
[yv,~,~,yvp] = lsim(sys,pv,uv);       
data_val_1 = lpviddata(yv,pv,uv,Ts);  % noisy data set
data_val_2 = lpviddata(yvp,pv,uv,Ts); % noise free data set 



%% LPV-ARX
am=1+p+p^2+p^3;
A_init={1, am, am};
B_init={am, am, am};
template_arx = lpvidpoly(A_init,B_init);
options = lpvarxOptions('Display','on');
m_arx = lpvarx(data_est, template_arx, options);
 
%% LPV-ARMAX
cm=1+p;
C_init={1, cm, cm};
template_armax = lpvidpoly(A_init, B_init,C_init);
opts = lpvpolyestOptions(...
    'Initialization', 'polypre', ...
    'Regularization', ...
        struct('Lambda', 1E-5), ...
    'SearchOptions', ...
        struct('StepSize', 1, 'StepTolerance', 1E-8, 'MaxIterations', 10), ...
    'Display', 'on');
m_armax = lpvpolyest(data_est, template_armax, opts);

%% LPV-IV estimate
template_iv = lpvidpoly([], m_arx.B,{1,1,1},{1,1,1},m_arx.A);
m_iv=riv(data_est, template_iv, 10^-4,200); 


%% LPV-OE
F_init=A_init;
opts = lpvpolyestOptions(...
    'Initialization', 'riv', ...
    'Regularization', ...
        struct('Lambda', 1E-5), ...
    'SearchOptions', ...
        struct('StepSize', 1, 'StepTolerance', 1E-8, 'MaxIterations', 10), ...
    'Display', 'on');
template_oe = lpvidpoly([], B_init,[],[],F_init);
m_oe = lpvpolyest(data_est, template_oe, opts);

%% LPV-BJ
D_init=C_init;
opts = lpvpolyestOptions(...
    'Regularization', ...
        struct('Lambda', 1E-5), ...
    'SearchOptions', ...
        struct('StepSize', 1, 'StepTolerance', 1E-8, 'MaxIterations', 10), ...
    'Display', 'on');
C_init{2}.matrices(:,:,1)=m_iv.C.Mat{2}.matrices(:,:,1);
C_init{3}.matrices(:,:,1)=m_iv.C.Mat{3}.matrices(:,:,1);
D_init{2}.matrices(:,:,1)=m_iv.D.Mat{2}.matrices(:,:,1);
D_init{2}.matrices(:,:,1)=m_iv.D.Mat{2}.matrices(:,:,1);
template_bj = lpvidpoly([], m_iv.B,C_init,D_init,m_iv.F);
m_bj = lpvpolyest(data_est, template_bj, opts);
 

%% Comparing results
figure;
compare(data_val_2,m_arx,m_armax,m_oe,m_iv,m_bj);
figure;
compare(data_val_1,m_arx,m_armax,m_oe,m_iv,m_bj,1);

