function tests = synthesisTest
%SYNTHESISTEST Test synthesis algorithms
tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(testCase) % Called once before the tests in this file are run
%% Define model CT
% Random model with 2 scheduling variables
rho1 = preal('rho1','ct','Range',[-1 1],'RateBound',[-.1,.1]);
rho2 = preal('rho2','ct','Range',[-2 0],'RateBound',[-.3,.2]);
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

synopt = lpvsynOptions('SolverOptions',sdpsettings('solver','sdpt3','verbose',0), ...
                       'Verbose',0);

testCase.TestData.ct.sys = sys;
testCase.TestData.ct.ny = ny;
testCase.TestData.ct.nu = nu;
testCase.TestData.ct.synopt = synopt;

% Grid-based model
grid = struct;
grid.rho1 = linspace(-1,1,5);
grid.rho2 = linspace(-2,0,3);

sysLocal = extractLocal(sys,grid);
sysGrid = lpvgridss(sysLocal,grid);

testCase.TestData.ct.sysGrid = sysGrid;

%% Define model CT where rho1 has inf rate bound
% Random model with 2 scheduling variables
rho1 = preal('rho1','ct','Range',[-1 1]);
rho2 = preal('rho2','ct','Range',[-2 0],'RateBound',[-.3,.2]);
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

testCase.TestData.ct2.sys = sys;
testCase.TestData.ct2.ny = ny;
testCase.TestData.ct2.nu = nu;
testCase.TestData.ct2.synopt = synopt;

%% Define model DT
% Random model with 2 scheduling variables
phi1 = preal('phi1','dt','Range',[-1 1],'RateBound',[-.1,.1]);
phi2 = preal('phi2','dt','Range',[-1 0],'RateBound',[-.3,.2]);
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

testCase.TestData.dt.sys = sys;
testCase.TestData.dt.ny = ny;
testCase.TestData.dt.nu = nu;
testCase.TestData.dt.synopt = synopt;

% Grid-based model
grid = struct;
grid.phi1 = linspace(-1,1,5);
grid.phi2 = linspace(-1,0,3);

sysLocal = extractLocal(sys,grid);
sysGrid = lpvgridss(sysLocal,grid);

testCase.TestData.dt.sysGrid = sysGrid;

testCase.TestData.ct.L2 = [];

%% Define model DT where rho1 has inf rate bound
% Random model with 2 scheduling variables
phi1 = preal('phi1','dt','Range',[-1 1]);
phi2 = preal('phi2','dt','Range',[-1 0],'RateBound',[-.3,.2]);
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

testCase.TestData.dt2.sys = sys;
testCase.TestData.dt2.ny = ny;
testCase.TestData.dt2.nu = nu;
testCase.TestData.dt2.synopt = synopt;
end

%% Test functions
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%            Passive             %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %
% Continuous-time
function testPassiveCtSynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'passive';
synopt.parameterVaryingStorage = 0;

[K,~] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
passiveCheck = lpvnorm(sysCL,'passive','parameterVaryingStorage',0,'verbose',0);

verifyTrue(testCase, passiveCheck);
end

function testPassiveCtGridSynthesis(testCase)
sys    = testCase.TestData.ct.sysGrid;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'passive';
synopt.parameterVaryingStorage = 0;

[K,~] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
passiveCheck = lpvnorm(sysCL,'passive','parameterVaryingStorage',0,'verbose',0);

verifyTrue(testCase, passiveCheck);
end

%%%%%%%%%%%%%%%%
% Discrete-time
function testPassiveDtSynthesis(testCase)
sys    = testCase.TestData.dt.sys;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'passive';
synopt.parameterVaryingStorage = 0;

[K,~] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
passiveCheck = lpvnorm(sysCL,'passive','parameterVaryingStorage',0,'verbose',0);

verifyTrue(testCase, passiveCheck);
end

function testPassiveDtGridSynthesis(testCase)
sys    = testCase.TestData.dt.sysGrid;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'passive';
synopt.parameterVaryingStorage = 0;

[K,~] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
passiveCheck = lpvnorm(sysCL,'passive','parameterVaryingStorage',0,'verbose',0);

verifyTrue(testCase, passiveCheck);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continuous-time projection
function testPassiveCtProjectionSynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'passive';
synopt.method = 'projection';
synopt.parameterVaryingStorage = 0;

[K,~] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
passiveCheck = lpvnorm(sysCL,'passive','parameterVaryingStorage',0,'verbose',0);

verifyTrue(testCase, passiveCheck);
end

function testPassiveCtGridProjectionSynthesis(testCase)
sys    = testCase.TestData.ct.sysGrid;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'passive';
synopt.method = 'projection';
synopt.parameterVaryingStorage = 0;

[K,~] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
passiveCheck = lpvnorm(sysCL,'passive','parameterVaryingStorage',0,'verbose',0);

verifyTrue(testCase, passiveCheck);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continuous-time parameter-varying storage
function testPassiveCtVaryingStorageSynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'passive';
synopt.parameterVaryingStorage = 1;

[K,~] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
passiveCheck = lpvnorm(sysCL,'passive','parameterVaryingStorage',1,'verbose',0);

verifyTrue(testCase, passiveCheck);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discrete-time parameter-varying storage
function testPassiveDtVaryingStorageSynthesis(testCase)
sys    = testCase.TestData.dt.sys;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'passive';
synopt.parameterVaryingStorage = 1;

[K,~] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
passiveCheck = lpvnorm(sysCL,'passive','parameterVaryingStorage',1,'verbose',0);

verifyTrue(testCase, passiveCheck);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%             Linf               %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %
% Continuous-time
function testLinfCtSynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'linf';
synopt.parameterVaryingStorage = 0;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);

sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'linf','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testLinfCtGridSynthesis(testCase)
sys    = testCase.TestData.ct.sysGrid;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'linf';
synopt.parameterVaryingStorage = 0;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);

sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'linf','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*2e-2);
end

function testLinfCtGridVsAffineSynthesis(testCase)
sys     = testCase.TestData.ct.sys;
sysGrid = testCase.TestData.ct.sysGrid;
ny      = testCase.TestData.ct.ny;
nu      = testCase.TestData.ct.nu;
synopt  = testCase.TestData.ct.synopt;

synopt.performance = 'linf';
synopt.parameterVaryingStorage = 0;

[~,gam] = lpvsyn(sys,ny,nu,synopt);
[~,gamGrid] = lpvsyn(sysGrid,ny,nu,synopt);

difGamma = abs(gam-gamGrid); 

verifyLessThanOrEqual(testCase, difGamma, 5e-7);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discrete-time
function testLinfDtSynthesis(testCase)
sys    = testCase.TestData.dt.sys;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'linf';
synopt.parameterVaryingStorage = 0;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'linf','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testLinfDtGridSynthesis(testCase)
sys    = testCase.TestData.dt.sysGrid;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'linf';
synopt.parameterVaryingStorage = 0;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'linf','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma);

verifyLessThanOrEqual(testCase, difGamma, gamma*5e-2);
end

function testLinfDtGridVsAffineSynthesis(testCase)
sys     = testCase.TestData.dt.sys;
sysGrid = testCase.TestData.dt.sysGrid;
ny      = testCase.TestData.dt.ny;
nu      = testCase.TestData.dt.nu;
synopt  = testCase.TestData.dt.synopt;

synopt.performance = 'linf';
synopt.parameterVaryingStorage = 0;

[~,gam] = lpvsyn(sys,ny,nu,synopt);
[~,gamGrid] = lpvsyn(sysGrid,ny,nu,synopt);

difGamma = abs(gam-gamGrid); 

verifyLessThanOrEqual(testCase, difGamma, 1e-6);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continuous-time projection
function testLinfCtProjectionSynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'linf';
synopt.method = 'projection';
synopt.parameterVaryingStorage = 0;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'linf','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testLinfCtGridProjectionSynthesis(testCase)
sys    = testCase.TestData.ct.sysGrid;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'linf';
synopt.method = 'projection';
synopt.parameterVaryingStorage = 0;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'linf','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testLinfCtGridVsAffineProjectionSynthesis(testCase)
sys     = testCase.TestData.ct.sys;
sysGrid = testCase.TestData.ct.sysGrid;
ny      = testCase.TestData.ct.ny;
nu      = testCase.TestData.ct.nu;
synopt  = testCase.TestData.ct.synopt;

synopt.performance = 'linf';
synopt.method = 'projection';
synopt.parameterVaryingStorage = 0;

[~,gam] = lpvsyn(sys,ny,nu,synopt);
[~,gamGrid] = lpvsyn(sysGrid,ny,nu,synopt);

difGamma = abs(gam-gamGrid); 

verifyLessThanOrEqual(testCase, difGamma, 2e-2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continuous-time parameter-varying storage
function testLinfCtVaryingStorageSynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'linf';
synopt.parameterVaryingStorage = 1;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'linf','parameterVaryingStorage',1,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discrete-time parameter-varying storage
function testLinfDtVaryingStorageSynthesis(testCase)
sys    = testCase.TestData.dt.sys;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'linf';
synopt.parameterVaryingStorage = 1;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'linf','parameterVaryingStorage',1,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*5e-2);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%              H2                %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %
% Continuous-time
function testH2CtSynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'h2';
synopt.parameterVaryingStorage = 0;

sys.D(1,1) = 0;   % Set Dzw to zero for H2;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'h2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testH2CtGridSynthesis(testCase)
sys    = testCase.TestData.ct.sysGrid;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'h2';
synopt.parameterVaryingStorage = 0;

sys.D(1,1,:) = 0;   % Set Dzw to zero for H2;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'h2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testH2CtGridVsAffineSynthesis(testCase)
sys     = testCase.TestData.ct.sys;
sysGrid = testCase.TestData.ct.sysGrid;
ny      = testCase.TestData.ct.ny;
nu      = testCase.TestData.ct.nu;
synopt  = testCase.TestData.ct.synopt;

synopt.performance = 'h2';
synopt.parameterVaryingStorage = 0;

sys.D(1,1) = 0;   % Set Dzw to zero for H2;
sysGrid.D(1,1,:) = 0;   % Set Dzw to zero for H2;

[~,gamma] = lpvsyn(sys,ny,nu,synopt);
[~,gammaGrid] = lpvsyn(sysGrid,ny,nu,synopt);
difGamma = abs(gamma-gammaGrid); 

verifyLessThanOrEqual(testCase, difGamma, 2e-5);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discrete-time
function testH2DtSynthesis(testCase)
sys    = testCase.TestData.dt.sys;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'h2';
synopt.parameterVaryingStorage = 0;

sys.D(1,1) = 0;   % Set Dzw to zero for H2;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'h2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testH2DtGridSynthesis(testCase)
sys    = testCase.TestData.dt.sysGrid;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'h2';
synopt.parameterVaryingStorage = 0;

sys.D(1,1,:) = 0;   % Set Dzw to zero for H2;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'h2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testH2DtGridVsAffineSynthesis(testCase)
sys     = testCase.TestData.dt.sys;
sysGrid = testCase.TestData.dt.sysGrid;
ny      = testCase.TestData.dt.ny;
nu      = testCase.TestData.dt.nu;
synopt  = testCase.TestData.dt.synopt;

synopt.performance = 'h2';
synopt.parameterVaryingStorage = 0;

sys.D(1,1) = 0;   % Set Dzw to zero for H2;
sysGrid.D(1,1,:) = 0;  

[~,gamma] = lpvsyn(sys,ny,nu,synopt);
[~,gammaGrid] = lpvsyn(sysGrid,ny,nu,synopt);
difGamma = abs(gamma-gammaGrid); 

verifyLessThanOrEqual(testCase, difGamma, 2e-8);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continuous-time projection
function testH2CtProjectionSynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'h2';
synopt.method = 'projection';
synopt.parameterVaryingStorage = 0;

sys.D(1,1) = 0;   % Set Dzw to zero for H2;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'h2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testH2CtGridProjectionSynthesis(testCase)
sys    = testCase.TestData.ct.sysGrid;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'h2';
synopt.method = 'projection';
synopt.parameterVaryingStorage = 0;

sys.D(1,1,:) = 0;   % Set Dzw to zero for H2;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'h2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testH2CtGridVsAffineProjectionSynthesis(testCase)
sys     = testCase.TestData.ct.sys;
sysGrid = testCase.TestData.ct.sysGrid;
ny      = testCase.TestData.ct.ny;
nu      = testCase.TestData.ct.nu;
synopt  = testCase.TestData.ct.synopt;

synopt.performance = 'h2';
synopt.method = 'projection';
synopt.parameterVaryingStorage = 0;

sys.D(1,1) = 0;   % Set Dzw to zero for H2;
sysGrid.D(1,1,:) = 0;

[~,gamma] = lpvsyn(sys,ny,nu,synopt);
[~,gammaGrid] = lpvsyn(sysGrid,ny,nu,synopt);
difGamma = abs(gamma-gammaGrid); 

verifyLessThanOrEqual(testCase, difGamma, 2e-2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continuous-time parameter-varying storage
function testH2CtVaryingStorageSynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'h2';
synopt.parameterVaryingStorage = 1;

sys.D(1,1) = 0;   % Set Dzw to zero for H2;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'h2','parameterVaryingStorage',1,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*2.5e-2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discrete-time parameter-varying storage
function testH2DtVaryingStorageSynthesis(testCase)
sys    = testCase.TestData.dt.sys;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'h2';
synopt.parameterVaryingStorage = 1;

sys.D(1,1) = 0;   % Set Dzw to zero for H2;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'h2','parameterVaryingStorage',1,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*2e-2);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%              L2                %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %
% Continuous-time
function testL2CtSynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'l2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testL2CtGridSynthesis(testCase)
sys    = testCase.TestData.ct.sysGrid;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'l2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testL2CtGridVsAffineSynthesis(testCase)
sys     = testCase.TestData.ct.sys;
sysGrid = testCase.TestData.ct.sysGrid;
ny      = testCase.TestData.ct.ny;
nu      = testCase.TestData.ct.nu;
synopt  = testCase.TestData.ct.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;

[~,gamma] = lpvsyn(sys,ny,nu,synopt);
[~,gammaGrid] = lpvsyn(sysGrid,ny,nu,synopt);
difGamma = abs(gamma-gammaGrid); 

verifyLessThanOrEqual(testCase, difGamma, 5e-4);
end

%%%%%%%%%%%%%%%%
% Discrete-time
function testL2DtSynthesis(testCase)
sys    = testCase.TestData.dt.sys;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'l2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testL2DtGridSynthesis(testCase)
sys    = testCase.TestData.dt.sysGrid;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'l2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testL2DtGridVsAffineSynthesis(testCase)
sys     = testCase.TestData.dt.sys;
sysGrid = testCase.TestData.dt.sysGrid;
ny      = testCase.TestData.dt.ny;
nu      = testCase.TestData.dt.nu;
synopt  = testCase.TestData.dt.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;

[~,gamma] = lpvsyn(sys,ny,nu,synopt);
[~,gammaGrid] = lpvsyn(sysGrid,ny,nu,synopt);
difGamma = abs(gamma-gammaGrid); 

verifyLessThanOrEqual(testCase, difGamma, 5e-4);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continuous-time projection
function testL2CtProjectionSynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'l2';
synopt.method = 'projection';
synopt.parameterVaryingStorage = 0;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'l2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testL2CtGridProjectionSynthesis(testCase)
sys    = testCase.TestData.ct.sysGrid;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'l2';
synopt.method = 'projection';
synopt.parameterVaryingStorage = 0;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'l2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testL2CtGridVsAffineProjectionSynthesis(testCase)
sys     = testCase.TestData.ct.sys;
sysGrid = testCase.TestData.ct.sysGrid;
ny      = testCase.TestData.ct.ny;
nu      = testCase.TestData.ct.nu;
synopt  = testCase.TestData.ct.synopt;

synopt.performance = 'l2';
synopt.method = 'projection';
synopt.parameterVaryingStorage = 0;

[~,gamma] = lpvsyn(sys,ny,nu,synopt);
[~,gammaGrid] = lpvsyn(sysGrid,ny,nu,synopt);
difGamma = abs(gamma-gammaGrid); 

verifyLessThanOrEqual(testCase, difGamma, 1e-2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discrete-time projection
function testL2DtProjectionSynthesis(testCase)
sys    = testCase.TestData.dt.sys;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'l2';
synopt.method = 'projection';
synopt.parameterVaryingStorage = 0;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'l2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testL2DtGridProjectionSynthesis(testCase)
sys    = testCase.TestData.dt.sysGrid;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'l2';
synopt.method = 'projection';
synopt.parameterVaryingStorage = 0;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'l2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testL2DtGridVsAffineProjectionSynthesis(testCase)
sys     = testCase.TestData.dt.sys;
sysGrid = testCase.TestData.dt.sysGrid;
ny      = testCase.TestData.dt.ny;
nu      = testCase.TestData.dt.nu;
synopt  = testCase.TestData.dt.synopt;

synopt.performance = 'l2';
synopt.method = 'projection';
synopt.parameterVaryingStorage = 0;

[~,gamma] = lpvsyn(sys,ny,nu,synopt);
[~,gammaGrid] = lpvsyn(sysGrid,ny,nu,synopt);
difGamma = abs(gamma-gammaGrid); 

verifyLessThanOrEqual(testCase, difGamma, 8e-3);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continuous-time parameter-varying storage
function testL2CtVaryingStorageSynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 1;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'l2','parameterVaryingStorage',1,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continuous-time parameter-varying storage in one parameter
function testL2CtVaryingStorageOneParameterSynthesis(testCase)
sys    = testCase.TestData.ct2.sys;
ny     = testCase.TestData.ct2.ny;
nu     = testCase.TestData.ct2.nu;
synopt = testCase.TestData.ct2.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 1;

[K,gamma,Xcl] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
[gammaNorm,XclNorm] = lpvnorm(sysCL,'l2','parameterVaryingStorage',1,'verbose',0);
difGamma = abs(gammaNorm-gamma);

% Check if storage function only varies in rho2
storageVariationCheck = sys.A.bfuncs{3} == Xcl.bfuncs{2} && Xcl.bfuncs{2} == XclNorm.bfuncs{2};

verifyTrue(testCase, (difGamma<=gamma*1e-2) && storageVariationCheck);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discrete-time parameter-varying storage
function testL2DtVaryingStorageSynthesis(testCase)
sys    = testCase.TestData.dt.sys;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 1;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'l2','parameterVaryingStorage',1,'verbose',0);
difGamma = abs(gammaNorm-gamma); 

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discrete-time parameter-varying storage in one parameter
function testL2DtVaryingStorageOneParameterSynthesis(testCase)
sys    = testCase.TestData.dt2.sys;
ny     = testCase.TestData.dt2.ny;
nu     = testCase.TestData.dt2.nu;
synopt = testCase.TestData.dt2.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 1;

[K,gamma,Xcl] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
[gammaNorm,XclNorm] = lpvnorm(sysCL,'l2','parameterVaryingStorage',1,'verbose',0);
difGamma = abs(gammaNorm-gamma);

% Check if storage function only varies in rho2
storageVariationCheck = sys.A.bfuncs{3} == Xcl.bfuncs{2} && Xcl.bfuncs{2} == XclNorm.bfuncs{2};

verifyTrue(testCase, (difGamma<=gamma*1e-2) && storageVariationCheck);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CT synthesis with pole constraints
function testL2CtPoleConstraintSynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;
pmat = poleConstraintOptions('rightHalfPlane',-200,'leftHalfPlane',-1,'circle',250,'sector',deg2rad(75));
synopt.poleConstraint = pmat;

K = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);

% Sample for 100 random scheduling-variables the eigenvalues of the
% closed-loop A-matrix
A = sysCL.A;
rng(1);
pSample = rand(100,2).*[diff(A.timemap.Range{1}),diff(A.timemap.Range{2})]+[A.timemap.Range{1}(1),A.timemap.Range{2}(1)];

Asample = squeeze(num2cell(feval(A,pSample),[1,2]));

eigValues = cellfun(@eig,Asample,'UniformOutput',false);
eigValues = cell2mat(eigValues);

% Check if the eigenvalues satisfy the constraints set by the pole
% constraints
Q = pmat.Q;
S = pmat.S;
T = pmat.T;
U = pmat.U;
R = T'*inv(U)*T; %#ok<MINV> 
P = [Q,S;S',R];

nQ = size(Q,1);
nR = size(R);

checkConstraint = @(z) all(eig([eye(nQ);z*eye(nR)]'*P*[eye(nQ);z*eye(nR)])<0);

checkEig = arrayfun(checkConstraint,eigValues);

verifyTrue(testCase, all(checkEig));
end

function testL2CtGridPoleConstraintSynthesis(testCase)
sys    = testCase.TestData.ct.sysGrid;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;
pmat = poleConstraintOptions('rightHalfPlane',-200,'leftHalfPlane',-1,'circle',250,'sector',deg2rad(75));
synopt.poleConstraint = pmat;

K = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);

% Sample at grid-points of the closed-loop A-matrix
A = sysCL.A;
Asample = squeeze(gridvar(A).matrices);

eigValues = cellfun(@eig,Asample,'UniformOutput',false);
eigValues = cell2mat(eigValues);

% Check if the eigenvalues satisfy the constraints set by the pole
% constraints
Q = pmat.Q;
S = pmat.S;
T = pmat.T;
U = pmat.U;
R = T'*inv(U)*T; %#ok<MINV> 
P = [Q,S;S',R];

nQ = size(Q,1);
nR = size(R);

checkConstraint = @(z) all(eig([eye(nQ);z*eye(nR)]'*P*[eye(nQ);z*eye(nR)])<0);

checkEig = arrayfun(checkConstraint,eigValues);

verifyTrue(testCase, all(checkEig));
end

function testL2CtLTIPoleConstraintSynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

sys = extractLocal(sys,mean(cell2mat(sys.SchedulingTimeMap.Range),2)');
sys = lpvss(sys);

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;
pmat = poleConstraintOptions('rightHalfPlane',-200,'leftHalfPlane',-1,'circle',250,'sector',deg2rad(75));
synopt.poleConstraint = pmat;

K = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);

% Get eigenvalues of closed-loop A-matrix (LTI)
A = sysCL.A.matrices;
eigValues = eig(A);

% Check if the eigenvalues satisfy the constraints set by the pole
% constraints
Q = pmat.Q;
S = pmat.S;
T = pmat.T;
U = pmat.U;
R = T'*inv(U)*T; %#ok<MINV> 
P = [Q,S;S',R];

nQ = size(Q,1);
nR = size(R);

checkConstraint = @(z) all(eig([eye(nQ);z*eye(nR)]'*P*[eye(nQ);z*eye(nR)])<0);

checkEig = arrayfun(checkConstraint,eigValues);

verifyTrue(testCase, all(checkEig));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CT synthesis with no direct feedthrough
function testL2CtNoDirectFeedThroughSynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;
synopt.directFeedthrough = 0;

K = lpvsyn(sys,ny,nu,synopt);

% Check if D-matrix is (almost) zero
verifyTrue(testCase, simplify(K.D,sqrt(eps)).matrices == 0);
end

function testL2CtGridNoDirectFeedThroughSynthesis(testCase)
sys    = testCase.TestData.ct.sysGrid;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;
synopt.directFeedthrough = 0;

K = lpvsyn(sys,ny,nu,synopt);

% Check if D-matrix is (almost) zero
verifyLessThanOrEqual(testCase, abs(getMatrices(K.D)), sqrt(eps));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DT synthesis with no direct feedthrough
function testL2DtNoDirectFeedThroughSynthesis(testCase)
sys    = testCase.TestData.dt.sys;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;
synopt.directFeedthrough = 0;

K = lpvsyn(sys,ny,nu,synopt);

% Check if D-matrix is (almost) zero
verifyTrue(testCase, simplify(K.D,sqrt(eps)).matrices == 0);
end

function testL2DtGridNoDirectFeedThroughSynthesis(testCase)
sys    = testCase.TestData.dt.sysGrid;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;
synopt.directFeedthrough = 0;

K = lpvsyn(sys,ny,nu,synopt);

% Check if D-matrix is (almost) zero
verifyLessThanOrEqual(testCase, abs(getMatrices(K.D)), sqrt(eps));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CT synthesis with different dependency
function testL2CtDependencySynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;
synopt.dependency = [0 0 0 0];

K = lpvsyn(sys,ny,nu,synopt);

% B, C, and D matrix should be constant
check = isconst(simplify(K.B,1e-10)) && isconst(simplify(K.C,1e-10)) && isconst(simplify(K.D,1e-10));
verifyTrue(testCase, check);
end

function testL2CtGridDependencySynthesis(testCase)
sys    = testCase.TestData.ct.sysGrid;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;
synopt.dependency = [0 0 0 0];

K = lpvsyn(sys,ny,nu,synopt);

% B, C, and D matrix should be constant
error = [norm(reshape(K.B,size(K.B,1),[])-K.B(:,:,1));...
         norm(reshape(K.C,size(K.C,2),[])-K.C(:,:,1)');...
         norm(reshape(K.D,size(K.D,1),[])-K.D(:,:,1))];

verifyLessThanOrEqual(testCase, error, 1e-10);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DT synthesis with different dependency
function testL2DtDependencySynthesis(testCase)
sys    = testCase.TestData.dt.sys;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;
synopt.dependency = [0 0 0 0];

K = lpvsyn(sys,ny,nu,synopt);

% B, C, and D matrix should be constant
check = isconst(K.B) && isconst(K.C) && isconst(K.D);
verifyTrue(testCase, check);
end

function testL2DtGridDependencySynthesis(testCase)
sys    = testCase.TestData.dt.sysGrid;
ny     = testCase.TestData.dt.ny;
nu     = testCase.TestData.dt.nu;
synopt = testCase.TestData.dt.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;
synopt.dependency = [0 0 0 0];

K = lpvsyn(sys,ny,nu,synopt);

% B, C, and D matrix should be constant
error = [norm(reshape(K.B,size(K.B,1),[])-K.B(:,:,1));...
         norm(reshape(K.C,size(K.C,2),[])-K.C(:,:,1)');...
         norm(reshape(K.D,size(K.D,1),[])-K.D(:,:,1))];

verifyLessThanOrEqual(testCase, error, 1e-10);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CT transformation without extra conditioning
function testL2CtNoCondSynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;
synopt.improveConditioning = 0;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'l2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma);

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testL2CtGridNoCondSynthesis(testCase)
sys    = testCase.TestData.ct.sysGrid;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;
synopt.improveConditioning = 0;

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'l2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma);

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CT projection without extra conditioning
function testL2CtNoCondProjSynthesis(testCase)
sys    = testCase.TestData.ct.sys;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;
synopt.improveConditioning = 0;
synopt.method = 'projection';

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'l2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma);

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end

function testL2CtGridNoCondProjSynthesis(testCase)
sys    = testCase.TestData.ct.sysGrid;
ny     = testCase.TestData.ct.ny;
nu     = testCase.TestData.ct.nu;
synopt = testCase.TestData.ct.synopt;

synopt.performance = 'l2';
synopt.parameterVaryingStorage = 0;
synopt.improveConditioning = 0;
synopt.method = 'projection';

[K,gamma] = lpvsyn(sys,ny,nu,synopt);
sysCL = lft(sys,K,nu,ny);
gammaNorm = lpvnorm(sysCL,'l2','parameterVaryingStorage',0,'verbose',0);
difGamma = abs(gammaNorm-gamma);

verifyLessThanOrEqual(testCase, difGamma, gamma*1e-2);
end