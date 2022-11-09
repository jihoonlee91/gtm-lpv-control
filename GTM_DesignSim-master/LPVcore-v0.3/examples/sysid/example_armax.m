%% In this example, we show the ARMAX identification scheme
clearvars; close all; clc; rng(1);

%% Example 1: LPV-ARMAX Identification
%
% LPV-ARMAX Identification with polynomial dependency. We take the
% identification setting and data-generating system of [1].

% Create the parameter-varying matrices
p1 = preal('pos', 'dt');

a1 = 1   - 0.5*p1 - 0.3*p1*p1;
a2 = 0.5 - 0.7*p1 - 0.5*p1*p1;

b1 = 0.5 - 0.4*p1 + 0.1*p1*p1;
b2 = 0.2 - 0.3*p1 - 0.2*p1*p1;

c1 = 0.9 * p1;
c2 = b2;
c3 = b1;

sys = lpvidpoly({1, a1,a2},{0,b1}, {1, c1, c2, c3}); % TODO: bug if first b1 == []
sysArx = lpvidpoly({1, a1, a2}, {0, b1});
sys.NoiseVariance = 0.1^2;
sysArx.NoiseVariance = sys.NoiseVariance;

N = 4000;
u = rand(N,1);
p = 0.2+0.4 * sin(0.035*pi*(1:N)')+ 0.3*rand(N,1);

[y,~,~,yp,v] = lsim(sys,p,u);       % Generate data including noise signal

fprintf('The SNR is %f dB\n', LPVcore.snr(yp,v)); 

% Create LPV data object and plot it
data = lpviddata(y,p,u,0.1);
plot(data);

% LPV-ARMAX identification. The original system is in the
% model set.
sysestArx = lpvarx(data, randlpvidpoly(sysArx));
options = lpvarmaxOptions('Display', 'on', ...
    'Regularization', struct('Lambda', 1E-7));
sysestArmax = lpvarmax(data, sysestArx, options);

y = lsim(sys, p, u, [], zeros(N, 1));
ys1Arx = lsim(sysestArx,p,u,[],zeros(N,1));        % Simulate sysest with no noise input
ys1Armax = lsim(sysestArmax, p, u, [], zeros(N, 1));

farx = bfr(y, ys1Arx);
farmax = bfr(y, ys1Armax);
fprintf('Fit: %f %% (ARX), %f %% (ARMAX)\n', farx, farmax);

figure;
    plot([y(1:200),ys1Arx(1:200),ys1Armax(1:200)]);
    title('Example 1: LPV-ARMAX. Output channel 1');
    xlabel('Time sample [-]');
    ylabel('Amplitude [-]');
    legend('Measured output', 'ARX model', 'ARMAX model')

% [1]:?Piga, D., Cox, P., T?th, R., & Laurain, V. (2015). LPV system
% identification under noise corrupted scheduling and output signal
% observations. Automatica, 53, 329?338.

%% Example 2: LPV-ARX Identification
%
% An example with scheduling variables with a time delay. LPV-ARX
% identification is performed with a template for the A and B polynomial.

% Create the parameter-varying matrices
p1 = preal('pos', 'dt');
p2 = pmatrix(1, 'BasisType', 'affine', ...
    'BasisParametrization', {}, ...
    'SchedulingTimeMap', timemap(-1, 'dt','Name', {'vel'}));
p3 = pmatrix(1, 'BasisType', 'affine', ...
    'BasisParametrization', {}, ...
    'SchedulingTimeMap', timemap(-2, 'dt','Name', {'pos'}));
 
ny = 2;  nu = 3; B10 = randn(ny,nu);
a1 = diag([-0.5,0.55])-0.1*p1;
a2 = diag([0.3,0.35])  +0.2*p2;
b1 = B10+0.1*p1+0.2*p3;
b2 = [diag([-0.25,0.25]),[0;0]]*p1+[[0;0],diag([0.1,0.15])]*p3;

% Create LPV-IDPOLY object. A(q,p(t)) y(t) = B(q,p(t)) u(t) + e(t).
sys = lpvidpoly({eye(2), a1, a2},{b1, b2});
sys.NoiseVariance = 0.02;

% Simulate LPV-IO object,
N = 1000;
u = randn(N,nu);
p = [sin((1:N)*0.1)', sin((1:N)*0.2)']+0.3*randn(N,2);

[y,~,~,yp,v] = lsim(sys,p,u);       % Generate data including noise signal

fprintf('The average SNR is %f\n', mean(LPVcore.snr(yp,v)) ); 

% Create LPV data object
data = lpviddata(y,p,u,0.1); 

plot(data);     % Show data

% LPV-ARX identification, unregularized. We create a template for the A
% and B polynomial. The original system is in the model set.
sysest = lpvidpoly({eye(2), a1, a2, a1, a2},{b1,b2,b1});
sysest = lpvarx(data,sysest);

ys1 = lsim(sysest,p,u,[],zeros(N,2));        % Simulate sysest with no noise input

figure;
subplot(2,1,1)
    plot([y(1:200,1),ys1(1:200,1)]);
    title('Example 2: LPV-ARX without regularization. Output channel 1');
    xlabel('Time sample [-]');
    ylabel('Amplitude [-]');
    legend('Measured output', 'Simulated output')
subplot(2,1,2)
    plot([y(1:200,2),ys1(1:200,2)]);
    title('Output channel 2');
    xlabel('Time sample [-]');
    ylabel('Amplitude [-]');
    legend('Measured output', 'Simulated output')


% LPV-ARX identification, regularized
options = lpvarxOptions;
optionsRegul = lpvarxRegulOptions;
sysest = lpvidpoly({eye(2), a1, a2, a1, a2},{b1,b2,b1});
sysest = lpvarx(data,sysest,options,optionsRegul);

ys1 = lsim(sysest,p,u,[],zeros(N,2));        % Simulate sysest with no noise input

figure;
subplot(2,1,1)
    plot([y(1:200,1),ys1(1:200,1)]);
    title('Example 2: LPV-ARX with regularization. Output channel 1');
    xlabel('Time sample [-]');
    ylabel('Amplitude [-]');
    legend('Measured output', 'Simulated output')
subplot(2,1,2)
    plot([y(1:200,2),ys1(1:200,2)]);
    title('Output channel 2');
    xlabel('Time sample [-]');
    ylabel('Amplitude [-]');
    legend('Measured output', 'Simulated output')
    