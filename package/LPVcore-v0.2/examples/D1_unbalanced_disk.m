%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 DC motor utilization demo with LPVcore
%                         Demo 1: LPV-IO models
%                          written by R. Toth
%                         version (25-06-2021)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clearvars; close all; clc;

if ~exist('pmatrix','class'); cd(['..',filesep]); end  % Toolbox is not added to Matlab search path. Go to root of the toolbox
 
%% Define model parameters
K=53.6*10^-3;   % [Nm/A] Motor torque constant
R=9.50;         % [Ohm] Motor resistance
L=0.84*10^-3;   % [H] Motor inductance
J=2.2*10^-4;    % [Nm^2] Complete disk inertia
b=6.6*10^-5;    % [Nms/rad] Friction coefficient
M=0.07;         % [kg] Additional mass
l=0.042;        % [m] Mass distance from the disk center
g=9.8;          % [m/s^2] Gravitational acceleration 
tau=(R*J)/(R*b+K^2);       % lumped back emf constant
Km=K/(R*b+K^2);            % lumped torque constant 

% For LPV model conversion we take advantage of that 
%
%         sinc(theta)*theta=sin(theta) 
%
% Hence, we can introduce a scheduling variable that is p=sinc(theta)
 
%% Create continuous-time coefficients [lower position model]
p = preal('p','ct');         % Define scheduling variable with name p
                            % 'CT' means continuous time
                            
                       
a0=(M*g*l/J)*p;        % coefficent with linear static dependence          
a1=1/tau;               % constant coefficent;
a2=1;                   % constant coefficent;

b0=Km/tau;              % constant coefficent;

% Coefficients of the CT LPV-IO model
% \sum_{i=0}^{n_a=2} a_i(p(t))* d^i/dt^i y(t) = \sum_{j=0}^{n_b=0} b_j(p(t))* d^j/dt^j u(t)
A=[a0,a1,a2];
B=[b0];
 
% The resulting matrices belong to the pmatrix object
% Observe the various properties
disp(A);

%% Possible operations
 
% Let's create another scheduling variable
 
sp=preal('speed','ct');
 
% Scalar case
% + - * with scheduling variables and division of pmatrix by constants 
 
disp(a0+p+sp+1)
disp(a0-p-sp-1)
disp(a0*p)
disp(a0*p*sp)
disp(a0/4)
% Matrix case
% + - * with matrices and division of pmatrix by constants 
disp(A+A)
disp(A-2*A)
disp(A'*A)
disp([A;A;A] / rand(3,3))
disp(A ./ rand(1,3))

%% Direct definition of parameter varying matrix functions
% To create the following polynomial parameter-varying matrix 
%   
%       S = [1 2] + [3 4] * p(t) + [5 6] * p(t)^2
%
disp(pmatrix( cat(3,[1 2], [3 4], [5 6]), ...
    'BasisType', 'poly', ...
    'BasisParametrization', {[], 1, 2}, ...
    'SchedulingName',{'p'}))
 
% To create
%       S = [1 2] + [3 4] * p(t) + [5 6] * p(t)*Speed(t)^2
%
% where m(t) = p(t) use
%
disp(pmatrix( cat(3,[1 2], [3 4],[5 6]), ...
    'BasisType', 'poly', ...
    'BasisParametrization', {[],[1 0],[1, 2]}, ...
    'SchedulingName',{'p','Speed'}))
 
% To create the following (custom) parameter-varying matrix 
%   
%       S = [1 2]* cos(p(t)) + [3 4] * sin(p(t)).^2 + [5 6]
%
% where m(t) = p(t) use
%
S=pmatrix( cat(3,[1 2], [3 4], [5 6]), ...
    'BasisType', 'custom', ...
    'BasisParametrization', ...
        {@(rho)cos(rho(:,1)), @(rho)sin(rho(:,1)).^2, []}, ...
    'SchedulingMap', 0, ...
    'SchedulingName', {'p'});
disp(S)
 
% Custom made dependencies can be also used directly without restrictions. 
% For example to evaluate the pmatrix at a given scheduling point, e.g. pi,
% we can use the freeze function and see what the last S will result in at
% p = pi.
 
freeze(S,pi)

%% Discrete time realisation of the DC motor
% Now let's develop a DT version of the motor model
 
Ts = 0.01;              % Sampling time
p  = preal('p','dt','Range', [-1/(2*pi-pi/2), 1]);  
                           % Define scheduling variable with name p
                           % 'dt' means discrete time     
                           % 'Range' defines range (min-max of the sinc)
                                 
p2= pmatrix(1, ...
    'SchedulingTimeMap',timemap([-2],'dt','Name',{'p'}));
                            % Dynamic dependence, shifted back 2 samples
 
% Alternatives:
p2= preal('p','dt','Dynamic',-2);  % Direct definition via preal
p2= pshift(p,-2);                     % By the Time Shift operator
                            
                       
a0=1;                       % Coefficients of the output side         
a1=Ts/tau-2;               
a2=1-Ts/tau+(M*g*l)/J*Ts^2*p2;  % Coefficient with dynamic dependence;
 
b2=Km/tau*Ts^2;             % Coefficient of the input side
 
% coefficients of the DT LPV-IO model
% \sum_{i=0}^{n_a=2} a_i(p(k))* y(k-i) = \sum_{j=0}^{n_b=0} b_j(p(k))* u(k-j)

%% Handling of dynamic dependence
 
% The timemap object
 
% A time-map is by default discrete-time, consisting of only p(t)
disp(timemap)
 
% To create the following discrete-time time-map: rho(t) =
% [p(t+5),p(t+1),p(t-2)]; use
disp(timemap([5,1,-2], 'dt'))
 
% To create the following continuous-time time-map: rho(t)=[p(t), d^2/dt^2
% p(t), d^1/dt^1 p(t)]; use
disp(timemap([0,2,1], 'ct'))
 
% timemap objects can be added together using UNION. For example
union(timemap(5, 'dt'), timemap([1 -2], 'dt'));
% provides a discrete-time time-map: rho(t) = [p(t+5),p(t+1),p(t-2)];
 
% To create the following discrete-time, affine parameter-varying matrix 
%   
%       A = [1 2]* p(t,1) + [3 4] * p(t-1,1) + [5 6] * p(t-1,2)         
%
% we will define rho(t) = [p(t); p(t-1)] via timemap([0 -1]), and we
% substitute rho(t) in the latter equation, which gives
%
%       A = [1 2]* rho(t,1) + [3 4] * rho(t,3) + [5 6] * rho(t,4).
%
% Hence, the original equation is obtained by using 
%
disp(pmatrix( cat(3,[1 2], [3 4], [5 6]), ...
    'BasisType', 'affine', ...
    'BasisParametrization', {1, 3, 4}, ...
    'SchedulingTimeMap',timemap([0 -1], 'dt', ...
        'Dimension', 2)))

%% Create a DT LPV-IO object

% Let's create our DT LPV-IO representation
sys_io = lpvio({1, a1 a2}, {0 0 b2}, Ts);  
sys_io.InputName = {'Voltage'};
sys_io.OutputName = {'Position'};


% To analyzie dimensions:
size(sys_io)


%% Simulate response
% Create some input and scheduling signals


N = 5000;
u = sin((1:N)*Ts*2)';
p_mag=0.25;
p=(1-p_mag)+p_mag*cos((1:N)*Ts)'; 

lsim(sys_io,p,u);
% Simulation from bottom position 

%% Create a CT LPV-SS object 

% Define the PV matrices
th = preal('p','ct','Range', [-1/(2*pi-pi/2), 1]) % Define CT scheduling 
A=[-1/tau, -(M*g*l/J)*th; 1 0];
B=[Km/tau; 0];
C=[0 1];
D=0;

sys_ss=lpvss(A,B,C,D);

T_end=N*Ts;
t=(Ts:Ts:T_end)';
lsim(sys_ss,p,u,t);

%% Create a DT LPV-SS object

% Define the PV matrices
th = preal('p','dt','Range', [-1/(2*pi-pi/2), 1]) % Define DT scheduling variable
A=eye(2)+Ts*[-1/tau, -(M*g*l/J)*th; 1 0];
B=Ts*[Km/tau; 0];

sys_ss=lpvss(A,B,C,D,Ts);

lsim(sys_ss,p,u); 

