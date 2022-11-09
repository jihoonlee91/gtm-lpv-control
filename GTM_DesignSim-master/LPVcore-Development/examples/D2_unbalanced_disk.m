%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 DC motor utilisation demo with LPVcore
%            Demo 2: Data generation and SySID with LPV-IO models
%                          written by R. Toth
%                         version (10-07-2021)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars; close all; clc;
rng(42);   % set random number generator seed so that results are reproducible
 
if ~exist('pmatrix','class'); cd(['..',filesep]); end  % Toolbox is not added to Matlab search path. Go to root of the toolbox
 
%% Define model parameters
K=53.6*10^-3;   % [Nm/A] Motor torque constant
R=9.50;         % [Ohm] Motor resistance
L=0.84*10^-3;   % [H] Motor inductance
J=2.2*10^-4;    % [Nm^2] Complete disk inertia
b=6.6*10^-5;    % [Nms/rad] Friction coefficient
M=0.07;         % [kg] Additional mass
l=0.042;        % [m] Mass distance from the disk center
g=9.8;          % [m/s^2] Gravitational acceleration 
tau=(R*J)/(R*b+K^2);       % lumped back emf constant
Km=K/(R*b+K^2);            % lumped torque constant 

%% Define DT motor model as an idpoly object

Ts = 0.01;              % Sampling time
th2= preal('p','dt','Dynamic',-2);
                            % Dynamic dependence, shifted back 2 samples
                        
a0=1;                       % Coefficients of the output side         
a1=Ts/tau-2;               
a2=1-Ts/tau+((M*g*l)/J)*(Ts^2)*th2;  

b2=Km/tau*Ts^2;             % Coefficient of the input side

sys = lpvidpoly({1, a1, a2},{0, 0, b2}); % Create and idpolyobject => ARX model
sys.Ts = Ts;                   % Sampling time
sys.InputName = {'Voltage'};   % Naming of the signals
sys.OutputName = {'Angular position'};
sys.NoiseVariance = 8.8604*10^-8;    % Specify noise variance


%% Create data sets
 
N=2000;         % Number of samples
p_mag=0.25;     % Scheduling variation
 
% Estimation data
u=idinput(N,'rgs');
p=(1-p_mag)+p_mag*idinput(N,'rbs',[0,0.05]);
% Note that for the sake of simplicity, 'p' is generated idependently from
% the scheduling map and the associated angular position. This is not a
% limitation of the toolbox, but used to simplify the example.
 
[y,~,~,yp] = lsim(sys,p,u);       % Generate data including noise signal
                                  % y - noisy output
                                  % yp - noise free output 
 
if ~batchStartupOptionUsed
    fprintf('The SNR is %f dB.\n', LPVcore.snr(yp,y-yp));  % Signal-to-noise ratio
end
 
data_est = lpviddata(y,p,u,Ts);      % Create an LPV data object
plot(data_est);                       % Show data
 
% Create a noisy and a noise free validation data set
uv=idinput(N,'rgs');
pv=(1-p_mag)+p_mag*idinput(N,'rbs',[0,0.05]);
[yv,~,~,yvp] = lsim(sys,pv,uv);       
data_val_1 = lpviddata(yv,pv,uv,Ts);  % noisy data set
data_val_2 = lpviddata(yvp,pv,uv,Ts); % noise free data set 


%% LPV-ARX identification
 
% Create specification of the model structure
th=preal('p','dt');
th1=pshift(th,-1);
th2=pshift(th,-2);
 
% dimension of the coefficients and their functional dependency creates a
% template for the model structure. The individual parameter values do not
% matter
am1=1+th+th1+th2;
am2=am1;
bm0=am1;
bm1=am1;
bm2=am1;
template_sys = lpvidpoly({1, am1, am2}, {bm0, bm1, bm2});

% perform ARX identification with the specified model structure
options = lpvarxOptions('Display','on');
m_arx = lpvarx(data_est, template_sys, options);

% Simulate sysest with no noise on the validation data
ys_arx = lsim(m_arx,pv,uv,[],zeros(N,1));        
fit_arx = bfr(yvp,ys_arx); % BFR score

% Simualtion response can be also obtained by using compare
[ys_arx,fit_arx]=compare(data_val_2,m_arx);

% For prediction use compare(data_val,m_arx,1)
 
figure;
plot([yvp, ys_arx]);
title('LPV-ARX estimation');
xlabel('Time sample [-]');
ylabel('Amplitude [-]');
legend('Measured output (noise free)', 'Simulated output')
 
% Compare the estimates 
disp(m_arx.A.Mat{1})
disp(sys.A.Mat{1})
 
disp(m_arx.A.Mat{2})
disp(sys.A.Mat{2})
 
disp(m_arx.A.Mat{3})
disp(sys.A.Mat{3})


%% More complicated noise structure
 
% To make our unbalanced disk example more realistic and challenging, let's 
% assume that the noise is a coloured out noise which is dependent on the 
% scheduling (i.e., the angular position) according to
 
c0=1;
c1=0.75*pshift(th2,2);
d0=1;
d1=-0.25+0.25*pshift(th2,2);
 
 
sys = lpvidpoly([],{0, 0, b2},{c0,c1},{d0,d1},{a0, a1, a2}); % Create and idpolyobject => BJ model
sys.Ts = Ts;                   % Sampling time
sys.InputName = {'Voltage'};   % Naming of the signals
sys.OutputName = {'Angular position'};
sys.NoiseVariance = 4.932*10^-4;    % Specify noise variance
 
%Create the data sets
[y,~,~,yp] = lsim(sys,p,u);  
data_est = lpviddata(y,p,u,Ts);       % noisy estimation data set
fprintf('The SNR is %f dB.\n', LPVcore.snr(yp,y-yp));  % Signal-to-noise ratio
[yv,~,~,yvp] = lsim(sys,pv,uv);       
data_val_1 = lpviddata(yv,pv,uv,Ts);  % noisy validation data set
data_val_2 = lpviddata(yvp,pv,uv,Ts); % noise-free validation data set 


%% Identification with various model structures
 
% Let's identify the resulting data-generating system with increasingly
% complex model structures:
 
% LPV-ARX estimate
template_arx = lpvidpoly({1, am1, am2}, {bm0, bm1, bm2});
options = lpvarxOptions('Display','on');
m_arx = lpvarx(data_est, template_arx, options);
 
% LPV-ARMAX
% Assume a moving average part for the noise model and initialise the 
% estimation from the ARX results
cm0=1;
cm1=0.5*randn(1,2)*[1,th]'; % same structure for the coefficients as in the ARX case
template_armax = lpvidpoly(m_arx.A, m_arx.B,{cm0,cm1});
opts = lpvpolyestOptions(...
    'Initialization', 'polypre', ...
    'Regularization', ...
        struct('Lambda', 1E-5), ...
    'SearchOptions', ...
        struct('StepSize', 1, 'StepTolerance', 1E-8, 'MaxIterations', 10), ...
    'Display', 'on');
m_armax = lpvpolyest(data_est, template_armax, opts);
 
% LPV-OE
% Assume an output noise model and initialise the estimation from the ARX
% results
template_oe = lpvidpoly([], m_arx.B,[],[],m_arx.A);
m_oe = lpvpolyest(data_est, template_oe, opts);
            
% LPV-BJ
% Finally, let's use a model structure capable to fully capture the noise
cm0=1;
cm1=0.5*ones(1,2)*[1,th]'; 
dm0=1;
dm1=0.25*ones(1,2)*[1,th]'; 
template_bj = lpvidpoly([], m_oe.B,{cm0,cm1},{dm0,dm1},m_oe.F);
opts = lpvpolyestOptions(...
    'Regularization', ...
        struct('Lambda', 1E-5), ...
    'SearchOptions', ...
        struct('StepSize', 1, 'StepTolerance', 1E-8, 'MaxIterations', 10), ...
    'Display', 'on');
%template_bj = lpvidpoly([], m_oe.B,m_armax.C,m_armax.A,m_oe.F);
m_bj = lpvpolyest(data_est, template_bj, opts);
            
% Note that automatic initialisation can be also used.
 
% Finally let's compare the estimation results 
 
% Comparison in terms of simulation error:
 
[ys_arx,fit_arx]=compare(data_val_2,m_arx);
[ys_armax,fit_armax]=compare(data_val_2,m_armax);
[ys_oe,fit_oe]=compare(data_val_2,m_oe);
[ys_bj,fit_bj]=compare(data_val_2,m_bj);

figure;
plot(yvp);
hold on;
plot(ys_arx);
plot(ys_armax);
plot(ys_oe);
plot(ys_bj);
grid on; title('Simulated model responses');
legend('true',sprintf('arx (%.2f %%)',fit_arx),sprintf('armax (%.2f %%)',fit_armax),sprintf('oe (%.2f %%)',fit_oe),sprintf('bj (%.2f %%)',fit_bj));
 
 
% Comparison in terms of prediction error:
 
[ys_arx,fit_arx]=compare(data_val_1,m_arx,1);
[ys_armax,fit_armax]=compare(data_val_1,m_armax,1);
[ys_oe,fit_oe]=compare(data_val_1,m_oe,1);
[ys_bj,fit_bj]=compare(data_val_1,m_bj,1);
 
figure;
plot(yv);
hold on;
plot(ys_arx);
plot(ys_armax);
plot(ys_oe);
plot(ys_bj);
grid on; title('Predicted model responses');
legend('true',sprintf('arx (%.2f %%)',fit_arx),sprintf('armax (%.2f %%)',fit_armax),sprintf('oe (%.2f %%)',fit_oe),sprintf('bj (%.2f %%)',fit_bj));
 
