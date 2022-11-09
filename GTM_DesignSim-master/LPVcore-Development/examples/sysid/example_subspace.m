clearvars; close all; clc; rng(1);
addpath(genpath('../../src'));
%% LPV subspace identification
% From: "Subspace identification of Bilinear and LPV systems for open- and closed-loop data"
% Example 7.3 (without Monte Carlo study, past window fixed to 3, with parameter-varying C and D)

% Increasing the past window lowers the bias and increases the variance.
past_window = 3;
% Sample to generate
N = 1000;

%% Data generating system
p = preal('p', 'dt');
A = [0.5, 0.5; -0.5, 0.5] + pshift(p, -20) * [0.2, 0.2; -0.2, 0.2];
B = [1; 0] + p * [1; 1];
C = [1, 1] + p * [1, 1];
D = 10 + 0 * p;
K = [1; 0] + p * [0; 1];

NoiseVariance = 0.5;  % Gives approx. 15 SNR
Ts = 1;  % [s]

sys = lpvidss(A, B, C, D, 'innovation', K, [], NoiseVariance, Ts);
nx = sys.Nx;
nu = sys.Nu;
ny = sys.Ny;
np = sys.Np;

% Generate identification input
Range = [-1, 1];
Band = [0, 0.8];
u = idinput([N, nu], 'rgs', Band, Range);
p = u;
[y, ~, ~, yp] = lsim(sys, p, u);
fprintf('The SNR is %f dB.\n', LPVcore.snr(yp,y-yp));

data = lpviddata(y, p, u);
plot(data);

% Estimate the system (the true system is only passed as template system
% for extracting the basis functions, the coefficients are not used).

% 1: Without kernelization
options = lpvsidOptions(...
    'Display', 'on', ...
    'PastWindowSize', past_window, ...
    'Kernelization', false);
sys_est_nokernel = lpvsid(data, sys, options);

% 2: With kernelization
options = lpvsidOptions(...
    'Display', 'on', ...
    'PastWindowSize', past_window, ...
    'Kernelization', true, ...
    'KernelizationRegularizationConstant', 1);
sys_est_kernel = lpvsid(data, sys, options);

% Generate validaton data
u_val = idinput([N, nu], 'rgs', Band, Range);
p_val = u_val;
y_val = lsim(sys, p_val, u_val, [], 0, zeros(N, ny));
data_val = lpviddata(y_val, p_val, u_val);
y_val_est_nokernel = lsim(sys_est_nokernel, p_val, u_val, [], 0, zeros(N, ny));
y_val_est_kernel = lsim(sys_est_kernel, p_val, u_val, [], 0, zeros(N, ny));

%% Plot results
figure;
compare(data_val, sys_est_nokernel, sys_est_kernel);

% Close-up
figure;
plot(y_val(1:100, :), 'k-'); hold on; grid on;
plot(y_val_est_nokernel(1:100, :), 'b--');
plot(y_val_est_kernel(1:100, :), 'r--');
xlabel('k');
title('First 100 steps');
legend('true', 'est', 'est (kernelized)');
fprintf('Non-kernelized\n\tBest-Fit-Rate: %.1f%%\n\tVariance-Accounted-For: %.1f%%\n', ...
    bfr(y_val, y_val_est_nokernel), vaf(y_val, y_val_est_nokernel));
fprintf('Kernelized\n\tBest-Fit-Rate: %.1f%%\n\tVariance-Accounted-For: %.1f%%\n', ...
    bfr(y_val, y_val_est_kernel), vaf(y_val, y_val_est_kernel));

%% Local functions
function f = vaf(y, yhat)
    % Variance Accounted For (VAF)
    f = max(0, 1 - var(y - yhat) / var(y)) * 100;
end