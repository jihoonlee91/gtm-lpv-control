%% In this example, we show the ARX identification scheme
%
%  This example requires to set the current working directory to the
%  root of the toolbox.
return; % TODO: does not work yet
clear all; close all; clc;

if ~exist('pmatrix','class'); cd(['..',filesep]); end  % Toolbox is not added to mathlab search path. Go to root of the toolbox

%% Example 1: LPV-ARX Identification
%
% LPV-ARX Identification with polynomial dependency. We take the
% identification setting and data-generating system of [1].


% Create the parameter-varying matrices
p1 = preal('pos', 'dt');

a1 = 1   - 0.5*p1 - 0.3*p1*p1;
a2 = 0.5 - 0.7*p1 - 0.5*p1*p1;

b1 = 0.5 - 0.4*p1 + 0.1*p1*p1;
b2 = 0.2 - 0.3*p1 - 0.2*p1*p1;


sys = lpvidpoly({1, a1,a2},{0,b1,b2});
sys.NoiseVariance = 0.01^2;

N = 4000;
u = rand(N,1);
p = 0.2+0.4 * sin(0.035*pi*(1:N)')+ 0.3*rand(N,1);

[y,~,~,yp,v] = lsim(sys,p,u);       % Generate data including noise signal

%fprintf('The SNR is %f\n', LPVcore.snr(yp,v)); 


    % Create LPV data object
data = lpviddata(y,p,u,0.1); 

plot(data);     % Show data


    % LPV-ARX identification, unregularized. The original system is in the
    % model set.
options = lpvarxOptions('Display','on');
sysest = lpvarx(data,[3 3],[],'poly',options);

ys1 = lsim(sysest,p,u,[],zeros(N,1));        % Simulate sysest with no noise input

figure;
    plot([y(1:200),ys1(1:200)]);
    title('Example 1: LPV-ARX. Output channel 1');
    xlabel('Time sample [-]');
    ylabel('Amplitude [-]');
    legend('Measured output', 'Simulated output')

% [1]:?Piga, D., Cox, P., T?th, R., & Laurain, V. (2015). LPV system
% identification under noise corrupted scheduling and output signal
% observations. Automatica, 53, 329?338.

%% Example 2: LPV-ARX Identification
%
% An example with scheduling variables with a time delay. LPV-ARX
% identification is prefored with a template for the A and B polynomial.

    % Create the parameter-varying matrices
p1 = preal('pos');
p2 = pmatrix(1,'affineBasis',[], 'SchedulingTimeMap', timemap(-1,false,'SchedulingName','vel'));
p3 = pmatrix(1,'affineBasis',[], 'SchedulingTimeMap', timemap(-2,false,'SchedulingName','pos'));
 
ny = 2;  nu = 3; B10 = randn(ny,nu);
a1 = diag([-0.5,0.55])-0.1*p1;
a2 = diag([0.3,0.35])  +0.2*p2;
b1 = B10+0.1*p1+0.2*p3;
b2 = [diag([-0.25,0.25]),[0;0]]*p1+[[0;0],diag([0.1,0.15])]*p3;


    % Create LPV-IDPOLY object. A(q,p(t)) y(t) = B(q,p(t)) u(t) + e(t).
sys = lpvidpoly({1, a1, a2},{b1,b2});
sys.NoiseVariance = 0.02;    % same as diag([0.3 0.3])

    % Simulate LPV-IO object,
N = 1000;
u = randn(N,nu);
p = [sin((1:N)*0.1)', sin((1:N)*0.2)']+0.3*randn(N,2);

[y,~,~,yp,v] = lsim(sys,p,u);       % Generate data including noise signal

fprintf('The average SNR is %f\n', mean(snr(yp,v)) ); 

    % Create LPV data object
data = lpviddata(y,p,u,0.1); 

plot(data);     % Show data


    % LPV-ARX identification, unregularized. We create a template for the A
    % and B polynomial. The original system is in the model set.
options = lpvarxOptions('Display','off');
sysest = lpvarx(data,{1, a1,a2,a1,a2},{b1,b2,b1},options);

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
optionsRegul = lpvarxRegulOptions;
sysest = lpvarx(data,sys.A,sys.B,options,optionsRegul);

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
    