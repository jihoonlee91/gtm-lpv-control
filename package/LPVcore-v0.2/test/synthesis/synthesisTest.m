function tests = synthesisTest
%SYNTHESISTEST Test synthesis algorithms
tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(testCase) % Called once before the tests in this file are run
%% Define model CT
% Random model with 2 scheduling variables
rho1 = preal('rho1','ct','Range',[-1 1]);
rho2 = preal('rho2','ct','Range',[-2 0]);
nu=1;
ny=1;
% System matrices
rng(42);% set random number generator seed so that results are reproducible

A = rand(3)-eye(3)+rho2*0.1634*rand(3);
Bw = [-4;1;0.1*rho2];
Bu = [1;2;0.1];
Cz = [1,-1/8,10*rho1];
Cy = [1,5,-2];
Dzw = 5;
Dzu = 0; 
Dyw = -1;

Ag = A;
Bg = [Bw Bu];
Cg = [Cz;Cy];
Dg = [Dzw,Dzu;Dyw,zeros(ny,nu)];
sys = lpvss(Ag,Bg,Cg,Dg);

synoptCT = lpvsynOptions('Backoff',1.01,...
                         'NumericalAccuracy',1e-8,...
                         'SolverOptions',sdpsettings('solver','sdpt3','verbose',0));

testCase.TestData.ct.sys = sys;
testCase.TestData.ct.ny = ny;
testCase.TestData.ct.nu = nu;
testCase.TestData.ct.synopt = synoptCT;

%% Define model DT
% Random model with 2 scheduling variables
phi1 = preal('phi1','dt','Range',[-1 1]);
phi2 = preal('phi2','dt','Range',[-1 0]);
nu=1;
ny=1;
% System matrices
rng(3);% set random number generator seed so that results are reproducible

T = randn(3);
A = T\(diag([-.23 0.5 .9])+diag([.5 0 0])*phi1+diag([0 .2 0])*phi2)*T;
Bw = [0;1;0];
Bu = [1;0;0.1];
Cz = [1,.5,-1];
Cy = [1,.2,.3];
Dzw = 1;
Dzu = 0; 
Dyw = 0;

Ag = A;
Bg = [Bw Bu];
Cg = [Cz;Cy];
Dg = [Dzw,Dzu;Dyw,zeros(ny,nu)];
sys = lpvss(Ag,Bg,Cg,Dg);

synoptDT = lpvsynOptions('Backoff',1.01,...
                         'NumericalAccuracy',1e-8,...
                         'SolverOptions',sdpsettings('solver','sdpt3','verbose',0));

testCase.TestData.dt.sys = sys;
testCase.TestData.dt.ny = ny;
testCase.TestData.dt.nu = nu;
testCase.TestData.dt.synopt = synoptDT;
end

%% Test functions
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%            Passive             %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %
function testPassiveSynthesis(testCase)
% CT
sysCT = testCase.TestData.ct.sys;
nyCT = testCase.TestData.ct.ny;
nuCT = testCase.TestData.ct.nu;
synoptCT = testCase.TestData.ct.synopt;
synoptCT.performance = 'passive';

[Kct,~] = lpvsyn(sysCT,nyCT,nuCT,synoptCT);
sysCLct = lft(sysCT,Kct,nuCT,nyCT);
difpasCT = lpvnorm(sysCLct, 'passive');

% DT
sysDT = testCase.TestData.dt.sys;
nyDT = testCase.TestData.dt.ny;
nuDT = testCase.TestData.dt.nu;
synoptDT = testCase.TestData.dt.synopt;
synoptDT.performance = 'passive';

[Kdt,~] = lpvsyn(sysDT,nyDT,nuDT,synoptDT);
sysCLdt = lft(sysDT,Kdt,nuDT,nyDT);
difpasDT = lpvnorm(sysCLdt, 'passive');

% Verify
verifyTrue(testCase, difpasCT && difpasDT);           % is passive?
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%              L1                %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %
function testL1Synthesis(testCase)
sys = testCase.TestData.ct.sys;
ny = testCase.TestData.ct.ny;
nu = testCase.TestData.ct.nu;
synoptCT = testCase.TestData.ct.synopt;
synoptCT.performance = 'l1';

% L1
[K,gamma_L1] = lpvsyn(sys,ny,nu,synoptCT);
sysCL = lft(sys,K,nu,ny);
gamma_cl = lpvnorm(sysCL, 'l1');
difl1 = abs(gamma_cl-gamma_L1); 

verifyTrue(testCase, difl1 <= 0.1*gamma_L1); % super conservative bound, |gamma_syn-gamma_cl|<0.1*gamma_syn
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%              H2                %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %
function testH2Synthesis(testCase)
% CT
sysCT = testCase.TestData.ct.sys;
nyCT = testCase.TestData.ct.ny;
nuCT = testCase.TestData.ct.nu;
synoptCT = testCase.TestData.ct.synopt;
synoptCT.performance = 'h2';

sysCT.D(1,1) = 0;   % Set Dzw to zero for H2;

[Kct,gammaCT] = lpvsyn(sysCT,nyCT,nuCT,synoptCT);
sysCLct = lft(sysCT,Kct,nuCT,nyCT);
gamma_clCT = lpvnorm(sysCLct, 'h2','SolverOptions',sdpsettings('verbose',0,'solver','sdpt3','sdpt3.gaptol',1e-6)); % Fix for now before improving synthesis LMI conditioning
difh2CT = abs(gamma_clCT-gammaCT); 

% DT
sysDT = testCase.TestData.dt.sys;
nyDT = testCase.TestData.dt.ny;
nuDT = testCase.TestData.dt.nu;
synoptDT = testCase.TestData.dt.synopt;
synoptDT.performance = 'h2';

sysDT.D(1,1) = 0;   % Set Dzw to zero for H2;

[Kdt,gammaDT] = lpvsyn(sysDT,nyDT,nuDT,synoptDT);
sysDLct = lft(sysDT,Kdt,nuDT,nyDT);
gamma_clDT = lpvnorm(sysDLct, 'h2','SolverOptions',sdpsettings('verbose',0,'solver','sdpt3','sdpt3.gaptol',1e-6)); % Fix for now before improving synthesis LMI conditioning
difh2DT = abs(gamma_clDT-gammaDT); 

% Verify
verifyTrue(testCase, (difh2CT <= 0.05) && difh2DT <= 0.05);    % gamma syn H2 and CL H2 equal?
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%              L2                %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %
function testL2Synthesis(testCase)
% CT
sysCT = testCase.TestData.ct.sys;
nyCT = testCase.TestData.ct.ny;
nuCT = testCase.TestData.ct.nu;
synoptCT = testCase.TestData.ct.synopt;
synoptCT.performance = 'l2';

[Kct,gammaCT] = lpvsyn(sysCT,nyCT,nuCT,synoptCT);
sysCLct = lft(sysCT,Kct,nuCT,nyCT);
gamma_clCT = lpvnorm(sysCLct, 'l2');
difl2CT = abs(gamma_clCT-gammaCT); 

% DT
sysDT = testCase.TestData.dt.sys;
nyDT = testCase.TestData.dt.ny;
nuDT = testCase.TestData.dt.nu;
synoptDT = testCase.TestData.dt.synopt;
synoptDT.performance = 'l2';

[Kdt,gammaDT] = lpvsyn(sysDT,nyDT,nuDT,synoptDT);
sysCLdt = lft(sysDT,Kdt,nuDT,nyDT);
gamma_clDT = lpvnorm(sysCLdt, 'l2');
difl2DT = abs(gamma_clDT-gammaDT); 

% Verify
verifyTrue(testCase, (difl2CT <= 0.05) && (difl2DT <= 0.05));    % gamma syn L2 and CL L2 equal?
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%         L2-Projection          %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %
function testL2pSynthesis(testCase)
sys = testCase.TestData.ct.sys;
ny = testCase.TestData.ct.ny;
nu = testCase.TestData.ct.nu;
synoptCT = testCase.TestData.ct.synopt;
synoptCT.l2Method = 0;
synoptCT.performance = 'l2';

% L2 proj
[K,gamma] = lpvsyn(sys,ny,nu,synoptCT);
sysCL = lft(sys,K,nu,ny);
gamma_cl = lpvnorm(sysCL, 'l2');
difl2p = abs(gamma_cl-gamma); 

verifyTrue(testCase, difl2p <= 0.05);   % gamma syn L2p and CL L2p equal? (p for projection)
end