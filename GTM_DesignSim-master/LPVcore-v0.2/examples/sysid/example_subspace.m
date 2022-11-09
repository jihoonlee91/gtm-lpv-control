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
K = zeros(2, 1) + p * zeros(2, 1);

NoiseVariance = 6;  % Gives approx. 15 SNR
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
options = lpvsidOptions(...
    'Display', 'on', ...
    'PastWindowSize', past_window);
sys_est = lpvsid(data, sys, options);

% Generate validaton data
u_val = idinput([N, nu], 'rgs', Band, Range);
p_val = u_val;
y_val = lsim(sys, p_val, u_val, [], 0, zeros(N, ny));
y_val_est = lsim(sys_est, p_val, u_val, [], 0, zeros(N, ny));

%% Plot results
figure;
plot(y_val(1:100, :), 'k-'); hold on; grid on;
plot(y_val_est(1:100, :), 'k--');
xlabel('k');
title('First 100 steps');
legend('true', 'est');
fprintf('Best-Fit-Rate: %.1f%%\nVariance-Accounted-For: %.1f%%\n', ...
    bfr(y_val, y_val_est), vaf(y_val, y_val_est));

%% Local functions
function f = vaf(y, yhat)
    f = max(0, 1 - var(y - yhat) / var(y)) * 100;
end