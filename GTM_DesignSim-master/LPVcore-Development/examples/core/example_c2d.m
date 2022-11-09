clearvars; close all; clc;

%% Example of c2d function (continuous to discrete conversion of models) 
% Based on example 1 of "On the Discretization of Linear Fractional Representations of LPV
% Systems" (2012) by R. Toth et al.

% Number of Monte Carlo trials (paper used N_trials = 100)
N_trials = 1;
% Store Mean-Squared Error (MSE) of each trial to compare
% discretization methods (rectangular, 2nd order polynomial, Tustin)
MSE_rect = NaN(1, N_trials);
MSE_poly = NaN(1, N_trials);
MSE_tustin = NaN(1, N_trials);

% Define CT model
A = [66, -136; ...
    116, -86];
B = [eye(2), [1; 1]];
C = [-58, 123; ...
    -10, 75; ...
    1, 1];
D = [zeros(2), ones(2, 1); ...
    -0.1, -0.1, 0.1];

G = ss(A, B, C, D, 0);
p = preal('p', 'ct');
Delta = blkdiag(p, p);

Td = 1E-4; % sampling time [seconds]

% Perform discretization
sysc = lpvlfr(Delta, G);
sysd_rect = c2d(sysc, Td, 'exact-zoh-euler');
sysd_poly = c2d(sysc, Td, 'exact-zoh-poly', 2);
sysd_tustin = c2d(sysc, Td, 'exact-zoh-tustin');

% Loop over the trials
for i=1:N_trials
rng(i);

%% Compare simulations
N = 1 / Td;
% White u
u = randn(N, sysc.Nu);
% Uniform distribution [-1, 1]
p = -1 + 2 * rand(N, sysc.Np);
t = Td * linspace(0, N-1, N).';
% Simulation
% Currently using adaptive step-size solver.
% Such a CT simulation is worse than the DT approximations.
% Need to use a fixed point solver with the same/better sampling time.
% ODE4 Runge-Kutta (Simulink solver) was used for the results in the paper.
% Step size: 10^-5 seconds (not sure)
% Otherwise, try 10^-8 seconds
% Check "calc_home" from the files sent by Roland
yc =        lsim(sysc, p, u, t, 'SolverInterpolationMode', 'accurate', ...
    'SolverOpts', struct('MaxStep', 1E-4));
yd_rect =   lsim(sysd_rect, p, u, t);
yd_poly =   lsim(sysd_poly, p, u, t);
yd_tustin = lsim(sysd_tustin, p, u, t);
% Calculate mean squared error
MSE_rect(i) = sum((yc - yd_rect).^2) / N;
MSE_poly(i) = sum((yc - yd_poly).^2) / N;
MSE_tustin(i) = sum((yc - yd_tustin).^2) / N;

fprintf('Trial %i / %i\t\tMSE: %.2d (rect)\t\t%.2d (poly)\t\t%.2d (tustin)\n', ...
    i, N_trials, MSE_rect(i), MSE_poly(i), MSE_tustin(i));

end

% Plot last trial
figure;
plot(t, yc, '-x', t, yd_rect, '--o', t, yd_poly, '--*', t, yd_tustin, '--x');
legend('ct', sprintf('dt rectangular (MSE=%.2d)', MSE_rect(i)), ...
    sprintf('dt 2nd-polynom (MSE=%.2d)', MSE_poly(i)), ...
    sprintf('dt tustin (MSE=%.2d)', MSE_tustin(i))); grid on;