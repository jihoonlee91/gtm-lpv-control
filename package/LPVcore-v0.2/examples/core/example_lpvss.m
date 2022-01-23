%% In this example, we show the ARX identification scheme
%
%  This example requires to set the current working directory to the
%  root of the toolbox

clear all; close all; clc;

if ~exist('pmatrix','class'); cd(['..',filesep]); end  % Toolbox is not added to mathlab search path. Go to root of the toolbox


%% Example 1: LPV-SS Simulation
%
% LPV-SS simulation We take the the discrete-time mass-spring-damper system
% of [1].
p1 = preal('Stiffness', 'DT');
p2 = preal('Damping', 'DT', 'Dynamic', -1);
Ts = 1/20;

A   = eye(2) + Ts*[0, 1; -(1+p1), -(1+p2)];
% Discrete-time mass-spring-damper system
sys = lpvss(A, [0;Ts], [1 0], [], Ts, ...
    'StateName', {'Position','Velocity'}, ...
    'StateUnit', {'m','m/s'}, ...
    'OutputName', 'Position', 'OutputUnit','m', ...
    'InputName','Actuator','InputUnit','N');

ndims(sys)
size(sys)
        
N = 1e4;
u = 0.2*ones(N,1);
p = 0.4*[sin((1:N)*0.005)', cos((1:N)*0.01)'];

% Simulate LPV-SS object.
lsim(sys,p,u);

% Simulate LPV-SS object with initial condition.
x0 = [10 -10];      % x(0)
p0 = [-0.4,  -0.4   % p(-1)
      -0.39, -0.3]; % p(0)
u0 = 5;             % u(0)
lsim(sys,p,u, [], x0)


% Simulate LPV-SS object
[y,t,x] = lsim(sys,p,u);
    figure;
    subplot(2,1,1);
    title('Example 1: LPV-SS. States');
    stairs(t,x); 
    legend(sys.StateName);
    xlabel(['Time [',sys.TimeUnit,']']);
    ylabel('Amplitude [-]');
    
    subplot(2,1,2);
    title('Output');
    stairs(t,y);
    legend(sys.OutputName);
    xlabel(['Time [',sys.TimeUnit,']']);
    ylabel('Amplitude [-]');

    
% [1]: Cox, P. B., Weiland, S., and T�th, R. (2018). Affine
% Parameter-Dependent Lyapunov Functions for LPV Systems with Affine
% Dependence. IEEE Transactions on Automatic Control.    