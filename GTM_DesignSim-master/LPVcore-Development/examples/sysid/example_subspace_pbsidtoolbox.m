clearvars; close all; clc; rng(1);
%% Comparing subspace implementation of LPVcore and PBSID-Toolbox
% The PBSID-Toolbox can be found here:
%   https://github.com/jwvanwingerden/PBSID-Toolbox
% Ensure that it is installed and added to the path.
% This script is based on Example 20 of PBSID-Toolbox.
if ~exist('idafflpv', 'file')
    error('PBSID-Toolbox must be installed');
end

n = 5;  % The order of the system
m = 2;  % The number of scheduling parameters (including constant term)
r = 1;  % The number of inputs
l = 1;  % The number of outputs

% System matrices
A1 = .1 * rand(n);
A2 = zeros(n);
B1 = ones(n, 1);
B2 = zeros(n, 1);
K = -0.01 * ones(n, 1);
C = [eye(l), zeros(l, n-l)];
D = zeros(l, r);
Alpv = [A1 A2];
Blpv = [B1 B2];
Clpv = [C zeros(l,n)];
Dlpv = [D zeros(l,r)];
Klpv = [K zeros(n,l)];

%% LPVcore model structure
rho = preal('rho', 'dt');
A = A1 + A2 * rho;
B = B1 + B2 * rho;
template_sys = lpvidss(A, B, C, D, 'innovation', K);

%% Open-loop identification experiment
% Simulation of the model in open loop

% Measured data from the scheduling parameters
N = 1000;  % number of data points
t = (0:1:(N-1))';
rho = rand(N,1);
mu = [ones(N,1) rho];

% Measured input data
u = randn(N, r);

% Simulation of the system without noise
M = idafflpv(Alpv,Blpv,Clpv,Dlpv,Klpv,[],1);
y0 = sim(M,u,t,mu(:,2:end));

% Simulation of the system with noise
e = 0.05.*randn(N,l);
y = sim(M,u,t,mu(:,2:end),e);
disp('Signal to noise ratio (SNR) (open-loop)')
snr(y,y0)

%% Check simulation LPVcore
[y_lpvcore, ~, ~, y0_lpvcore] = lsim(template_sys, mu(:, 2), u, t, 0, e);
% Noise-free output
figure;
plot(t, y0(:, 1)); hold on; grid on;
plot(t, y(:, 1));
plot(t, y0_lpvcore(:, 1));
plot(t, y_lpvcore(:, 1));
xlabel('time'); ylabel('output');
legend('PBSID (noise-free)', 'PBSID (noise)', ...
    'LPVcore (noise-free)', 'LPVcore (noise)');

%%
% Identification of the model in open loop

% Defining a number of constants
p = 6;     % past window size
f = 6;     % future window size

% LPV identification
[~,x] = lordvarx(u,y,mu,f,p,'none','gcv',[0 0 0]);
x = lmodx(x,n);
[Aid,Bid,Cid,Did,Kid] = lx2abcdk(x,u,y,mu,f,p,[0 0 0]);
[Aid1,Bid1,Cid1,Did1,Kid1] = lx2abcdk(x,u,y,mu,f,p,[0 0 0],1);

Mi = idafflpv(Aid,Bid,Cid,Did,Kid,[],1);

%% Check subspace id LPVcore
data = lpviddata(y, mu(:, 2:end), u);
options = lpvsidOptions('PastWindowSize', p, ...
    'Display', 'off', 'Kernelization', true);
Mi_lpvcore = lpvsid(data, template_sys, options);

%% Validation results
% Validation data from the scheduling parameters
rho = rand(N,1);
mu = [ones(N,1) rho];

% Validation data
u = randn(N, r);

% Simulation of the system without noise
y0 = sim(M,u,t,mu(:,2:end));

% Simulation of identified LPV system
yid = sim(Mi,u,t,mu(:,2:end));
disp('VAF of identified LPV system')
vaf(y0,yid)

%% Check simulation identifified LPVcore system
[~, ~, ~, yid_lpvcore] = lsim(Mi_lpvcore, mu(:, 2:end), u);
disp('VAF of identified LPV system (LPVcore)')
vaf(y0, yid_lpvcore)