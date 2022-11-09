%% Example script for LPVIDOBF object
clearvars; clc; close all; rng(1);

% Pole location of inner function
v = [0.7, 0.8, 0.9];
% Number of extensions of inner function
ne = 3;

%% Parameterize scheduling dependence
% Dependence on the scheduling dependence enters the LPVIDOBF model
% structure through 2 filters: the Wiener and Hammerstein filters. These
% are placed before and after the linear time-invariant OBF bank,
% respectively.
% First, create the scheduling signals.
p = preal('p', 'dt');
% Then, determine the parametrization of the Wiener and Hammerstein
% filters. Each filter may be set to empty ("[]") to exclude it from being
% used. Note that a scalar expression will be expanded to match the number
% of coefficients (ne * numel(v) + 1).
% Wiener filter
% TODO: allow nb-by-1 and (nb-1)-by-1 (the coefficient corresponding to the static
% basis function is set to 1).
Wf = [];
% Hammerstein filter
Hf = [0, ones(1, ne * numel(v)) * 2 + p].';


%% Create LPVIDOBF object
% An LPVIDOBF object is a special instance of an LPVIDSS object. Therefore,
% all properties of that object, such as InputName, NoiseVariance and
% OutputName, can be used directly to construct LPVIDOBF objects.
NoiseVariance = .01;
Ts = 1;  % seconds
sys = lpvidobf(v, ne, Wf, Hf, NoiseVariance, Ts, ...
    'InputName', 'u', 'OutputName', 'y');

%% Analyze LPVIDOBF object
% Check that the pole locations and multiplicities are as specified
disp('Pole locations of OBFs (including multiplicities:');
disp(pole(extractLocal(sys, ones(sys.Np))));

%% Identification of LPVIDOBF objects
N = nparams(sys, 'free') * 100;
u = randn(N, sys.Nu);
p = randn(N, sys.Np);
x0 = randn(sys.Nx, 1);
y = lsim(sys, p, u, [], x0);
data_est = lpviddata(y, p, u, sys.Ts);
% Create template model structure (from which the scheduling dependence,
% state order and IO size are determined). The parameter values are not
% used in the identification.
template_sys = sys;
options = lpvobfestOptions('InitialState', 'estimate');
[est_sys, est_x0] = lpvobfest(data_est, template_sys, options);

%% Compare estimated model, template model and data
figure(1);
compare(data_est, est_sys); grid on;
disp('Relative 2-norm error of estimated initial state:');
disp(norm(est_x0 - x0) / norm(x0));

%% Validation
u_val = randn(N, sys.Nu);
p_val = randn(N, sys.Np);
y_val = lsim(sys, p_val, u_val, [], x0);
data_val = lpviddata(y_val, p_val, u_val, sys.Ts);
figure(2);
% TODO: Compare script should also estimate the initial condition
compare(data_val, est_sys); grid on;