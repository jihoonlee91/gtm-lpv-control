%% LPV-PEM identification
clearvars; close all; clc;
rng(1);

%% Example 1: Simple LPV-PEM identification
%
% Create the parameter-varying matrices
p1 = preal('pos', 'dt');

A = {1, 0.1 * p1};
b1 = 0.5 - 0.4*p1 + 0.1*p1*p1;
b2 = 0.2 - 0.3*p1 - 0.2*p1*p1;
B = {0, b1, b2};
C = {1, 1 - 0.3*p1 - 0.2*p1*p1};
D = {1, 1 - 0.1*p1};
f1 = 1   - 0.5*p1 - 0.3*p1*p1;
f2 = 0.5 - 0.7*p1 - 0.5*p1*p1;
F = {1, f1, f2};

% Data-generating system (ground truth)
sysGt = lpvidpoly({1}, B, C, D, F);
sysGt.NoiseVariance = 0.08^2;

% Generate model structures
sysOe = lpvidpoly({1}, B, {1}, {1}, F);
sysArx = lpvidpoly(F, B);
sysArmax = lpvidpoly(F, B, C);
sysBj = lpvidpoly({1}, B, C, D, F);

N = 400;
u = rand(N,1);
p = 0.2+0.4 * sin(0.035*pi*(1:N)')+ 0.3*rand(N,1);

%% Generate data
[y,~,~,yp,v] = lsim(sysGt, p, u);

fprintf('The SNR is %f\n', LPVcore.snr(yp,v)); 

% Create LPV data object
data = lpviddata(y,p,u,0.1); 

plot(data);     % Show data

%% Do identification
options = lpvarxOptions('Display','on');
sysestOe = lpvoe(data, randlpvidpoly(sysOe), ...
    lpvoeOptions('SearchMethod', 'pseudols', 'Display', 'on'));
sysestArx = lpvarx(data, randlpvidpoly(sysArx), ...
    options);
sysestArmax = lpvarmax(data, randlpvidpoly(sysArmax), ...
    lpvarmaxOptions('SearchMethod', 'pseudols', 'Display', 'on'));
sysestBj = lpvbj(data, randlpvidpoly(sysBj), ...
    lpvbjOptions('SearchMethod', 'pseudols', 'Display', 'on'));

y = lsim(sysGt, p, u, [], zeros(N, 1));
ys1Oe = lsim(sysestOe, p, u, [], zeros(N, 1));
ys1Arx = lsim(sysestArx, p, u, [], zeros(N, 1));
ys1Armax = lsim(sysestArmax, p, u, [], zeros(N, 1));
ys1Bj = lsim(sysestBj, p, u, [], zeros(N, 1));

foe = bfr(y, ys1Oe);
farx = bfr(y, ys1Arx);
farmax = bfr(y, ys1Armax);
fbj = bfr(y, ys1Bj);
fprintf('The fit is %f (OE) vs. %f (ARX) vs. %f (ARMAX) vs. %f (BJ)\n', ...
    foe, farx, farmax, fbj);

figure;
    plot([y(1:200),ys1Oe(1:200),ys1Arx(1:200),ys1Armax(1:200),ys1Bj(1:200)]);
    title('Example 1: LPV-OE. Output channel 1');
    xlabel('Time sample [-]');
    ylabel('Amplitude [-]');
    legend('Measured output', 'OE', 'ARX', 'ARMAX', 'BJ')

%% Example 2: LPV-BJ Identification with Regularization
%
% An example of LPV-BJ identification with an overparametrized model
% structure. The effect of tuning the regularization parameter Lambda is
% demonstrated by looking at the fit on a validation data set.
rng(1); clearvars; close all;

p = preal('p', 'dt');
noiseVariance = 0.1;

sys = lpvidpoly(1, [1, 2], ...
    {1, 0.1 * p^2, 0.5, 0.6 * p}, {1, 0, 0, 0, 0.4}, {1, 0, -0.1 * p, 0.1}, noiseVariance);
% For the template system, we will use an overparametrized BJ system for which
% each term of C, D and F depends affinely on p^1 and p^2.
t = 1 + p + p^2;
T = {1, t, t, t, t};  % a monic term and 4 delay terms
template_sys = lpvidpoly(1, [1, 2], ...
    T, T, T, noiseVariance);

% Create training and validation datasets
N = 100;
u = randn(N, sys.Nu);
p = randn(N, sys.Np);
y = lsim(sys, p, u);
data = lpviddata(y, p, u);

uVal = randn(N, sys.Nu);
pVal = randn(N, sys.Np);
yVal = lsim(sys, pVal, uVal);
dataVal = lpviddata(yVal, pVal, uVal);

% Values of lambda to iterate over
lambda = [0.1, 1, 10, 100];
fit = NaN(size(lambda));

for i=1:numel(lambda)
    fprintf('Regularized LPV-BJ identification with Lambda = %.1f\n', ...
        lambda(i));
    
    % Regularization is parametrized through the "Regularization"
    % structure. For more information, see LPVBJOPTIONS.
    regularization.Lambda = lambda(i);

    sysest = lpvbj(data, template_sys, lpvbjOptions('SearchMethod', 'pseudols', 'Display', 'on', ...
        'Regularization', regularization, ...
        'SearchOptions', struct('MaxIterations', 10)));

    [~, fit(i)] = compare(dataVal, sysest);
    fprintf('Fit: %.1f%%\n', max(fit(i), 0));
end

semilogx(lambda, max(fit, 0), 'k-*'); grid on;
title('Fit to validation data');
ylabel('Fit (%)');
xlabel('\lambda');
    