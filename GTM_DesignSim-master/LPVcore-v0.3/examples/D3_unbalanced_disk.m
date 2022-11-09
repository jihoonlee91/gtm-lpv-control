%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 DC motor utilisation demo with LPVcore
%            Demo 2: Data generation and SySID with LPV-SS models
%                          written by R. Toth
%                         version (10-07-2021)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rng(1);
clearvars; close all;

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
%% Create a DT - SS representation

th = preal('p','dt');       % Define DT scheduling variable
A=eye(2)+Ts*[-1/tau, -(M*g*l/J)*th; 1 0];
B=Ts*[Km/tau; 0];
C=[0 1];
D=0;
K = [0.108; 0.108];
sys_ss=lpvidss(A,B,C,D,'innovation',K, [], 0.001^2,Ts);
sys_ss.InputName = {'Voltage'};      % Naming of the signals
sys_ss.OutputName = {'Position'};


%% Create data sets
 
N=2000;         % Number of samples
p_mag=0.25;     % Scheduling variation
 
% Estimation data
u=idinput(N,'rgs');
p=(1-p_mag)+p_mag*idinput(N,'rbs',[0,0.05]);
% Note that for the sake of simplicity, 'p' is generated independently from
% the scheduling map and the associated angular position. This is not a
% limitation of the toolbox, but used to simplify the example.
 
[y,~,~,yp] = lsim(sys_ss,p,u);    % Generate data including noise signal
                                  % y - noisy output
                                  % yp - noise free output 
                                  
fprintf('The SNR is %f dB.\n', LPVcore.snr(yp,y-yp));  % Signal-to-noise ratio                            
                                  
 
data_est = lpviddata(y,p,u,Ts);      % Create an LPV data object
plot(data_est);                      % Show data
 
% Create a noisy and a noise free validation data set
uv=idinput(N,'rgs');
pv=(1-p_mag)+p_mag*idinput(N,'rbs',[0,0.05]);
[yv,~,~,yvp] = lsim(sys_ss,pv,uv);       
data_val_1 = lpviddata(yv,pv,uv,Ts);  % noisy data set
data_val_2 = lpviddata(yvp,pv,uv,Ts); % noise free data set 
 
 
%% LPV subspace identificaiton
n=2;           % model order
p_window=3;    % past window
template_ss=lpvidss(ones(n,n)+ones(n,n)*th,ones(n,1)+ones(n,1)*th,ones(1,n),ones(1,1),'innovation',ones(2,1), [], [],Ts);

options = lpvsidOptions('Display', 'on', ...
    'PastWindowSize', p_window);
m_ss_sid = lpvsid(data_est, template_ss, options);
 
% Plotting validation results 
[ys_ss_sid,fit_ss_sid]=compare(data_val_2,m_ss_sid);  % Simulation
 
f1=figure;
plot([yvp, ys_ss_sid]);
title('Simulated model response');
xlabel('Time sample [-]');
ylabel('Amplitude [-]');
leg1=[]; leg1{1}='Measured output (noise free)'; leg1{2}=sprintf('SS-SID (%.2f %%)',fit_ss_sid);
legend(leg1);
 
[ys_ss_sid,fit_ss_sid]=compare(data_val_1,m_ss_sid,1);  % Prediction
 
f2=figure;
plot([yv, ys_ss_sid]);
title('Predicted model response');
xlabel('Time sample [-]');
ylabel('Amplitude [-]');
leg2=[]; leg2{1}='Measured output'; leg2{2}=sprintf('SS-SID (%.2f %%)',fit_ss_sid);
legend(leg2);
 
 
%% LPV-PEM SS identification
 
% As a next step, let's use LPV-SS PEM. Note that this method uses a
% nonlinear optimisation to minimise the prediction error of LPV-SS model
% estimate. While it is capable to achieve consistent estimation of LPV-SS
% models compared to LPV subspace methods, it requires efficient
% initialisation. For this purpose we will use the previously identified
% model by the subspace method to initialise the estimation.
 
% First set the template structure to be similar as what we used for
% subspace:
template_ss=m_ss_sid; 
 
% Use default gradient search with DDLC, without specifying automatic
% initialisation the estimation will be initialised by the parameters of the
% template, i.e. in this case with the subspace estimate.
options = lpvssestOptions('Display','on');
m_ss_pem= lpvssest(data_est, template_ss, options);
 
% Plotting validation results 
[ys_ss_pem,fit_ss_pem]=compare(data_val_2,m_ss_pem);  % Simulation
 
figure(f1);
hold on;
plot(ys_ss_pem);
leg1{3}=sprintf('SS-PEM (%.2f %%)',fit_ss_pem);
legend(leg1);
 
[ys_ss_pem,fit_ss_pem]=compare(data_val_1,m_ss_pem,1);  % Prediction
 
figure(f2);
hold on;
plot(ys_ss_pem);
leg2{3}=sprintf('SS-PEM (%.2f %%)',fit_ss_pem);
legend(leg2);