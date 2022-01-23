% Demonstrate LPV-LFR modeling and identification
clearvars; clc; close all;
rng(42);

%% Create systems
G1 = drss(4, 4, 4);
G2 = drss(4, 4, 4);
G1.Ts = 1;
G2.Ts = 1;
p = preal('p', 'dt');
q = preal('q', 'dt');
D1 = p * eye(2);
D2 = q * randn(2);

sys1 = lpvlfr(D1, G1);
sys2 = lpvlfr(D2, G2);

sys1.InputName = {'u1', 'u2'};
sys2.InputName = {'u1', 'u2'};

%% Evaluate PLUS
sys = sys1 + sys2;
N = 100;
[y,t] = lsim(sys1,0.01 * randn(N, sys1.Np),ones(N, sys1.Nu));

figure;
plot(t, y);
xlabel('Time (seconds)');
ylabel('Amplitude');
title('LPV Simulation Results');
            
%% Identification