% run installPackage before running main for the first time

close all;
clear all;
clc;

rng('shuffle');

%% 0. Introduction
%
%   Welcome to LPVcore!
%   This tutorial shows the most important concepts in the toolbox.
%   After following this tutorial, you'll be able to 
%       (a) construct an LPV model of a mass-spring-damper system.
%       (b) simulate the time-behaviour of the model under excitation.
%       (c) identify an LPV model from simulation data.
%       (d) synthesise a stabilising controller for your LPV model.
%   After completing the tutorial you may also run the demo scripts D1-D3
%   for the unbalanced disk example located in this folder.
%
%   We will use the following abbreviations:
%       LPV Linear Parameter-Varying
%       CT  Continuous Time
%       DT  Discrete Time       
 
%% 0. Quick test to verify path
% To install LPVcore, it suffices to add the "src" directory to your MATLAB
% path. For some functionality, additional toolboxes are required. Consult
% README.md for more information.
try
    preal('p');
catch
    error('Please ensure the "src" directory is added to your MATLAB path.');
end

% Commands have a simple help, but documentation also available via, e.g.:
% doc preal

%% 1. Creating a scheduling variable
% Create a scheduling variable in continuous time (CT) / discrete time (DT)
% using PREAL
p = preal('p', 'ct');
q = preal('q', 'dt');
disp(p);

% Note that 'p' is a very simple instance of the PMATRIX class used to
% represent scheduling dependent coefficients or matrices. This class will 
% be discussed in more detail later in the tutorial. For now, focus on the 
% TIMEMAP property of p. A TIMEMAP is an object that translates a 
% scheduling variable to an "extended" scheduling rho to represent so 
% called dynamic dependence. In this case, the  "extended" scheduling is 
% the same as the original scheduling signal. However, some LPV 
% representations contain derivatives (CT) or time-shifted (DT) versions of
% the scheduling. To maintain a consistent representation of the LPV system
% regardless of operations that produce such dynamic dependence of the 
% coefficients, the scheduling is mapped to the extended scheduling 
% variable using the TIMEMAP object,
% i.e.:
%
%   [p(t), p(t-1), p(t-2)] ---> TIMEMAP ---> [rho1(t), rho2(t), rho3(t)]
%
% Differentiated or time-shifted versions of the scheduling can be obtained
% in several ways:
%  (i)   by calling PDIFF (CT) or PSHIFT (DT) on the scheduling

pd = pdiff(p, 1);
qd = pshift(q, 1);

%   (ii)  at construction time via preal 
pdd = preal('p', 'ct', 'Dynamic', 1);

%   (iii) directly modifying the timemap (only for advanced users, not 
%         treated here).
 

% The range and unit of a scheduling variable can be specified
x = preal('x', 'ct', ...
    'Range', [-1, +1], ...
    'Unit', 'cm');

%% 2. Parameter-varying matrices
%
% As we have seen, a scheduling variable is an instance of the PMATRIX 
% class. The PMATRIX class is a generalisation of a single scheduling 
% variable, i.e., 1*p(t), to parameter-varying scalar coefficients and even
% matrices of the following form:
%
%   A(p(t)) = A1 * phi_1(p(t)) + ... + An * phi_n(p(t))
%
% where A1, ..., An are regular matrices and phi_1, ..., phi_n are basis
% functions that map the (extended) scheduling rho to a scalar.
%
% The easiest way to create them is by combining a PREAL with a regular
% double. For example, the following line creates a PMATRIX with linear 
% static dependence on p(t):

A = randn(2) * p

% Besides the TIMEMAP, which we have seen in the previous section, PMATRIX
% has two other important properties: 'matrices' and 'bfuncs'. The former
% contains the constant matrices A1, ..., An in a 3D-array. The 'bfuncs'
% property stores the basis functions phi_1, ..., phi_n in a cell array.
% Currently, the following basis functions are supported:
%   1. PBCONST: phi(rho) = 1
%   2. PBAFFINE: phi(rho) = rho_i
%   3. PBPOLY: phi(rho) = rho_1^(d1) * ... * rho_m^(dm)
%   4. Custom made basis functions, e.g. trigonometrical, are also allowed
% The PMATRIX we created in the last line, has a single PBAFFINE basis
% function:

disp(A.bfuncs{1});

% PMATRIX objects can be manipulated and combined using various operators.
2 * A           % scalar and matrix multiplication
A * A
A .* A          % element-wise multiplication
A + A           % addition and subtraction
A - 2 * A
kron(A, A)      % Kronecker product
A'              % transposition
A.'
[A, A; A, A]    % horizontal and vertical concatenation
blkdiag(A, A) 
A^2             % exponentiation
A.^2 

% Note that performing arithmetics on a PMATRIX will automatically transform
% the basis functions in an appropriate way. For example, multiplying two
% PMATRIX objects composed of affine basis functions of the same scheduling
% variable results in a polynomial PMATRIX object (i.e, with PBPOLY basis 
% functions).

A = preal('p') * preal('p');
disp(A.bfuncs{1});

% The value of a PMATRIX at a certain frozen scheduling value can be
% extracted. In this operation, derivatives and time-shifted versions of p 
% are handled automatically.

A_f = freeze(A, 1)

% Sometimes, a PMATRIX contains many terms. For example, when the following
% PMATRIX is created for arbitrary degree d:
%
%   P = a_0 + a_1 p + a_2 * p^2 + ... + a_d * p^d
%
% In this case, the BasisType parameter can be specified as 'poly'
% (polynomial) and the exact coefficient values of the expression can be 
% given as an array. Example with coefficients a_0=1, a_1=2, a_3=3, etc.:
d = 5;
P_pol = pmatrix(reshape(1:d, 1, 1, d), 'BasisType', 'poly');
fplot(@(p) freeze(P_pol, p), [-1, 1]);
% Note: FPLOT currently throws a warning about properly vectorizing the
% function. This will be implemented in a future version.

% For more information on creating PMATRIX objects, see
% examples/core/example_pmatrix.m.

%% 3. LPV system representations
%
% Using parameter-varying matrices, parameter-varying system
% representations can be constructed. The LPVSS class implements LPV
% system representations in a state-space form with basis-affine 
% scheduling-dependent state-space matrices. The LPVIO class implements LPV
% system in an input-output (filter) form with basis-affine 
% parameter-dependent input-output coefficients.
%
% In the following example, a mass-spring-damper system with
% position-dependent spring and damper coefficients is modelled, both with 
% an LPV-SS representation and with an LPV-IO representation. 
%
% LPV-SS representation in continuous time:
x = preal('x', 'ct', 'Unit', 'm', 'Range', [-0.1, 0.1]);
kd = 1 + x^2;   % damper coefficient
ks = 1;         % spring coefficient
m = 1;          % mass
% LPV-SS model
A = [-kd / m, -ks / m; 1, 0];
B = [1 / m; 0];
C = [0, 1];
D = 0;
sysSS = lpvss(A, B, C, D);

% To show more functionalites we will develop a model of the system in DT 
% using an Euler discretisation. 
%
% LPV-IO model is considered in a filter form and it is parametrised by two
% cell arrays, A and B, of PMATRIX  objects. Each element of the cell array
% corresponds to a different order of the backward time-shift operator, 
% i.e.:
%
%   A_{i+1}(p) q^{-i}
%
% where q^{-i}u(k) = u(k-i).
% A filters the output, B filters the input. A must be monic (A{1} = 1).

Ts=0.01;
xd = preal('x', 'dt', 'Unit', 'm', 'Range', [-0.1, 0.1]); % DT scheduling
kd = 1 + xd^2;   % damper coefficient in DT
a0=1;
a1=pshift(kd,-2)/m*Ts-2;
a2=ks/m*Ts^2-kd/m*Ts+1;

A = {a0, a1, a2};   
B = {0, 0, Ts^2/m};
sysIO = lpvio(A, B, Ts);

% The CT version would be:
% a0=1;
% a1=kd/ks;
% a2=m/ks;
%  
% A = {a0, a1, a2};   
% B = {1};
% sysIO = lpvio(A, B);

% To verify that the two representations are equivalent, we perform a
% time-domain simulation using randomly generated data for the input and
% parameter.
N = 10;
NT=floor(1/Ts);
t = 0:Ts:(N-Ts);
u = kron(randn(N, 1),ones(NT,1));
p = kron(rand(N, 1),ones(NT,1));
t=t(1:length(u));

[ySS, tSS] = lsim(sysSS, p, u, t);
[yIO, tIO] = lsim(sysIO, p, u, t);

figure;
subplot(2, 1, 1);
plot(tSS, yIO, 'k--*'); grid on; title('LPV-IO in discrete time');
subplot(2, 1, 2);
plot(tSS, ySS, 'k--*'); grid on; title('LPV-SS in continuous time');
xlabel('t (s)'); ylabel('position (m)');

%% 4. Application domain: system identification
%
% LPVcore has algorithms for identifying LPV models from data.
% Broadly speaking, these algorithms can be divided into two classes:
%   1. Local methods: based on local linearizations of the underlying
%   system, i.e. the scheduling signal is kept constant during experiments.
%   2. Global methods: the scheduling signal is varied during the
%   experiments.
% In this tutorial, a global method for the identification of an LPVIDPOLY
% model is discussed. This model type is similar to IDPOLY from the System
% Identification toolbox, with scheduling-dependent coefficients. For an
% example of a local method to identify an LPV-SS model, see 
% examples/example_hinfid.m.
%

% First, a simple ARX data-generating system is defined.
p = preal('p', 'dt');
A = {1, 0.5 * p}; 
B = {1 + p, 1 - p};
sysDG = lpvidpoly(A, B);
sysDG.NoiseVariance = 0.1;

% Two datasets are generated (train and test) with N number of samples.
N = 200;
uTrain = randn(N, 1);
pTrain = rand(N, 1);
yTrain = lsim(sysDG, pTrain, uTrain);
dataTrain = lpviddata(yTrain, pTrain, uTrain);
uTest = randn(N, 1);
pTest = rand(N, 1);
yTest = lsim(sysDG, pTest, uTest, [], zeros(N, 1));
dataValid = lpviddata(yTest, pTest, uTest);

% Two algorithms will be explored to estimate this ARX model.
% First, a least-Squares approach based ARX estimation method using LPVARX.
% Then, a gradient search approach is used to estimate an ARMAX model with
% using LPVPOLYEST.
 
% Note that the optimisation problem is convex for ARX models, furthermore 
% the ARMAX parametrisation includes ARX models and the involved estimation 
% method is consistent therefore both methods asymptotically converge to a 
% global optimum.
%
 
% The model structure for the estimation methods is defined through a
% template system. The exact value of the parameters in the template is not
% important only the corresponding structure of the dependence that  
% is defined by them. This is used by the algorithms to estimate the 
% coefficients (i.e., the parameters).
 
% Both methods are given an LPVIDPOLY template with random coefficients, 
% but the same structure as the data-generating system (RANDLPVIDPOLY).
% Note that LPVARX does not make use of the value of the coefficients 
% (it only uses the structure). However, in case of LPVPOLYEST, the
% parameter values can be used as initial seeds for the optimisation. 
% The later is the default option, and one can see that convergence really
% depends on the initialisation. For automatic initialisation, see the help
% for polyest and demo D2.

AST = {1, 1 + p}; 
BST = {1 + p, 1 - p};
CST = {1, 1 + p};

struct_arx= lpvidpoly(AST, BST);
sysID1 = lpvarx(dataTrain, randlpvidpoly(struct_arx));

struct_armax= lpvidpoly(AST, BST, CST);
opts = lpvpolyestOptions('Display', 'on', ...
    'Regularization', struct('Lambda', 0.01), ...
    'SearchOptions', struct('StepSize', 10));
sysID2 = lpvpolyest(dataTrain, randlpvidpoly(struct_armax), opts);


[ys_arx,arx_fit]=compare(dataValid,sysID1,1);
[ys_armax,armax_fit]=compare(dataValid,sysID2,1);

fprintf('Fit LPV-ARX: %.1f%%\nFit LPV-ARMAX: %.1f%%\n', ...
    arx_fit, armax_fit);

figure;
plot(yTest);
hold on;
plot(ys_arx);
plot(ys_armax);
grid on; title('Simulated model responses');
legend('true','arx','armax');
