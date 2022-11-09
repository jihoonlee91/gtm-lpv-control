%%
%%  LPVcore System Identification tutorial
%%
%#ok<*VUNUS>
close all; clearvars;
rng(1); % for reproducibility

%% 0. Introduction
%
%   This tutorial shows the algorithms that are availabe for system
%   identification.
%   After following this tutorial, you'll know ...
%       ... which types of system identification algorithms are available.
%       ... how to use model representations for system identification.
%       ... how to identify a model from data and validate it.
%

%% 1. Simple data-generating system
%
% At first, we'll consider a simple discrete-time data-generating system with a single
% scheduling variable, p, which can be represented using an LPV-BJ (Box
% Jenkins) model structure. We'll generate two datasets: one for performing
% the identification and one for validating identified models. The
% following approaches will be used to identify the system:
%   1. Global identification using Prediction-Error-Method (PEM) with a
%   Pseudo-Linear Least Squares approach.
%   2. Global identification using PEM with a gradient descent approach.
%

p = preal('p', 'dt');
A = {1};
B0 = 0.5 + p + p^2;
B1 = -1 + p^2;
B2 = 0.25 + 0.5 * p;
B = {B0, B1, B2};
C1 = p; C2 = p^2;
C = {1, C1, C2};
D1 = p - p^2;
D = {1, D1};
F1 = 0.1 + 0.3 * p; F2 = -0.4 - 0.1 * p + 0.5 * p^2;
F = {1, F1, F2};

% Next, we'll set the output noise variance and sampling time
NoiseVariance = 0.05;
Ts = 1; % [s]

% We create the model using LPVIDPOLY, which is similar to IDPOLY.
dg_sys = lpvidpoly(A, B, C, D, F, NoiseVariance, Ts);

% For the global optimization methods, we create a dataset
N = 100;
uData = randn(N, dg_sys.Nu);
pData = rand(N, dg_sys.Np);
% A noise signal with the specified NoiseVariance 
[yData, ~, eData] = lsim(dg_sys, pData, uData);
fprintf('SNR is %.1f dB\n', LPVcore.snr(yData, eData));
% The signals are collected in LPVIDDATA, similar to IDDATA.
data = lpviddata(yData, pData, uData);
% A second, noiseless dataset is created to assess model quality
uValData = randn(N, dg_sys.Nu);
pValData = rand(N, dg_sys.Np);
[yValData, t] = lsim(dg_sys, pValData, uValData, [], zeros(N, 1));
val_data = lpviddata(yValData, pValData, uValData);

% An initial system is constructed by using RANDLPVIDPOLY.
% This function takes an LPVIDPOLY object and returns another LPVIDPOLY
% object with the same model structure and parameter dependence, but with
% random coefficient values.
init_sys = randlpvidpoly(dg_sys);

%% 1b. Creating more complicated data-generating systems
%% MIMO
% MIMO systems are supported for identification using the pseudo-linear LS
% approach, but not gradient descent (this will be added in a future
% version). To create a MIMO system, simply pass matrices instead of
% scalars into the polynomial terms A, B, C, D and F. The following example
% shows how to define a MIMO LPV-ARX system of the following form:
%
%   (I + A1 * p * z^{-1} ) y(t) = (B0 * p + B1 * p * z^{-1}) * u(t)
%
A1 = randn(2); B0 = randn(2); B1 = randn(2);
A_mimo = {eye(2), A1 * p};
B_mimo = {B0 * p, B1 * p};
sys_mimo = lpvidpoly(A_mimo, B_mimo);

%% Input delay
% System with an input delay can be modeled by setting the InputDelay
% property which is common to all LPV system representations in LPVcore
% (besides LPVIDPOLY, these include LPVSS, LPVIO and others). The following
% example shows how to define a MIMO LPV-ARX system with unit input delay:
%
%   (I + A1 * p * z^{-1} ) y(t) = (B0 * p + B1 * p * z^{-1}) * u(t - 1)
%
sys_mimo.InputDelay = 1;        % scalar input delay is expanded
sys_mimo.InputDelay = [1; 1];   % equivalent to previous line
sys_mimo.InputDelay = [1; 2];   % different delay per channel

%% Polynomial dependence on scheduling signal
% Models with a more complicated dependency on the scheduling signal can be
% created by creating more elaborate PMATRIX objects. This example shows
% how to construct an LPV-ARX system with Na = 1 and Nb is a
% hyper-parameter. Each term in B depends on the scheduling signal through
% some unknown polynomial in temperature ('T'). Only the maximum degree,
% d, of this polynomial is known.
% Order of the LPV-ARX system
nb = 3;
% Max. degree of the polynomial basis function
d = 4;
% Create the polynomial PMATRIX with specified degree and name
b_pol = pmatrix(ones(1, 1, d), ...
    'BasisType', 'poly', ...
    'SchedulingName', {'T'});
% Assign the polynomial to each term in B.
B_pol = cell(1, nb + 1);
B_pol(1, :) = {b_pol};
% Create the template system. Note that the values of the coefficients (in
% this case, they are all 1) are not used for LPVARX, but they are used for
% LPVPOLYEST. Thus, if a better initial estimate is available, that one
% should be used instead.
sys_pol = lpvidpoly({1}, B_pol, 'InputDelay', 1);

clearvars A1 B0 B1 A_mimo B_mimo sys_mimo B_pol b_pol sys_pol;

%% 2. Global PEM using Pseudo-Linear Least Squares
% Global PEM using a pseudo-linear least squares is implemented for the
% following discrete-time model structures:
%   LPVARX, LPVARMAX, LPVOE, LPVBJ
% Each of these functions needs only two input arguments:
%   data: an LPVIDDATA object containing the measured data.
%   template_sys: a template system of the corresponding model structure
%   from which the parameter-dependence is extracted, as well as the free
%   and non-free parameters.
options = lpvbjOptions('SearchMethod', 'pseudols', 'Display', 'on');
ls_sys = lpvbj(data, init_sys, options);

% The COMPARE function is implemented for LPVIDPOLY objects.
% We use it to see how the fit is improved from the random initial system
% to the identified model.
[y_ls_sys, fit_ls] = compare(val_data, ls_sys);

%% 3. Global PEM using Gradient Descent
grad_sys = lpvbj(data, ls_sys, ...
    lpvbjOptions(...
        'SearchMethod', 'gradient', ...  % <-- 'gradient' is the default
        'Display', 'on', ...
        'Regularization', struct('Lambda', 1E-4)));
[y_grad_sys, fit_grad] = compare(val_data, grad_sys);

%% 4. Compare systems
figure;
plot(t, yValData); hold on; grid on;
plot(t, y_ls_sys, 'k--');
plot(t, y_grad_sys, 'g--');
xlabel(['Time (', dg_sys.TimeUnit, ')']);
ylabel('Amplitude');
title('Simulated Response Comparison');
legend('Measured', ...
    sprintf('Pseudo-LS (fit: %.2f%%)', fit_ls), ...
    sprintf('Grad. descent refinement (fit: %.2f%%)', fit_grad));

%% More topics
% Regularized estimation: examples/sysid/example_pem_ls