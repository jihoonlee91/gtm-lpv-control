close all; clearvars; clc; rng(1);
%% Refined-IV method for identification of LPV Box-Jenkins models
%
% This example script demonstrates the functionality of the LPVRIV command,
% which implements the identification approach proposed in:
%
%   "Refined instrumental variable methods for identification of LPV
%   Box-Jenkins models" by V. Laurain et al. (2010)
%

%% Define data generating system and template model structure
% IO size
ny = 2;
nu = 2;
% Scheduling signal
p = preal('p', 'dt');
% Process part
B = {randn(ny, nu) + p * randn(ny, nu)};
F = {eye(ny), 0.1 * eye(ny) + p * diag([0.1, -0.05])};
% Noise part (must be scheduling-independent)
C = eye(ny);
D = eye(ny);
% Create LPVIDPOLY object
NoiseVariance = 1;
Ts = 1;
sys = lpvidpoly([], B, C, D, F, NoiseVariance, Ts, 'ZeroIsNonFree', true);
sys.OutputName = {'yA', 'yB'};
% Construct template system object from which scheduling dependency is extracted 
% (alternatively, an ARX pre-initialization is done by LPVRIV to find
% initial values for the iterative process).
template_sys = sys;

%% Estimation
% Create dataset for estimation
N = 100;  % num. time samples
u = randn(N, sys.Nu);
p = rand(N, sys.Np);
[y, t, ~, yNoiseFree] = lsim(sys, p, u);
fprintf('SNR is %.2f dB\n', LPVcore.snr(y, yNoiseFree - y));

max_iter = 4;
data = lpviddata(y, p, u);
[sys_est, pvec, theta] = lpvriv(data, template_sys, ...
    lpvrivOptions(...
        'Initialization', 'polypre', ...
        'Display', 'on', ...
        'MaxIterations', max_iter));

%% Validation
u_val = randn(N, sys.Nu);
p_val = rand(N, sys.Np);
y_val = lsim(sys, p_val, u_val, [], zeros(size(y)));
data_val = lpviddata(y_val, p_val, u_val);
% Simulation error
figure;
compare(data_val, sys_est); grid on;