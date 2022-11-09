function tests = lpvnormTest
%LPVNORMTEST Verify lpvnorm algorithms for LTI systems
tests = functiontests(localfunctions);
end

%% Setup
function setupOnce(testCase)
% Systems to compare LTI and LPV (affine)
rng(1)

% Two uncoupled mass-spring-damper system
T1 = randn(2);
A1 = [0 1;-rand,-rand];
B1 = [0;1];
C1 = [0 1];
D1 = 0;

sys1 = ss2ss(ss(A1,B1,C1,D1),T1);

T2 = randn(2);
A2 = [0 1;-rand,-rand];
B2 = [0;1];
C2 = [0 1];
D2 = 0;

sys2 = ss2ss(ss(A2,B2,C2,D2),T2);

sys = blkdiag(sys1,sys2);
lpvsys = lpvss(sys);

lpvsys1 = lpvss(sys1);

% Add data to testcase
testCase.TestData.sys1 = sys1;
testCase.TestData.sys = sys;
testCase.TestData.lpvsys = lpvsys;
testCase.TestData.lpvsys1 = lpvsys1;

% Systems to compare LPV affine and LPVgrid-based
rng(1)
% Affine CT
p_bounds = [-2 3;-1 2];

nx = 2;
nu = 1;
ny = 1;

p1 = preal('p1','Range',p_bounds(1,:));
p2 = preal('p2','Range',p_bounds(2,:));

A = -5*eye(nx)+randn(nx)*p1+randn(nx)*p2;
B = randn(nx,nu)+randn(nx,nu)*p1+randn(nx,nu)*p2;
C = randn(ny,nx)+randn(ny,nx)*p1+randn(ny,nx)*p2;
D = 3+randn(ny,nu)*p1+randn(ny,nu)*p2;

lpvsys2 = lpvss(A,B,C,D);

ng1 = 5;
ng2 = 3;

p1_grid = linspace(p_bounds(1,1),p_bounds(1,2),ng1);
p2_grid = linspace(p_bounds(2,1),p_bounds(2,2),ng2);

grid.p1 = p1_grid;
grid.p2 = p2_grid;

sys = extractLocal(lpvsys2,grid);

lpvsys3 = lpvgridss(sys,grid);

% Add data to testcase
testCase.TestData.lpvsys2 = lpvsys2;
testCase.TestData.lpvsys3 = lpvsys3;

% Affine DT
rng(3);
pd1 = preal('p1','DT','Range',p_bounds(1,:));
pd2 = preal('p2','DT','Range',p_bounds(2,:));

Ad = .1*randn(nx)+.1*randn(nx)*pd1+.1*randn(nx)*pd2;
Bd = randn(nx,nu)+randn(nx,nu)*pd1+randn(nx,nu)*pd2;
Cd = randn(ny,nx)+randn(ny,nx)*pd1+randn(ny,nx)*pd2;
Dd = 12+randn(ny,nu)*pd1+randn(ny,nu)*pd2;

lpvsys4 = lpvss(Ad,Bd,Cd,Dd,-1);

sysd = extractLocal(lpvsys4,grid);

lpvsys5 = lpvgridss(sysd,grid);

% Add data to testcase
testCase.TestData.lpvsys4 = lpvsys4;
testCase.TestData.lpvsys5 = lpvsys5;
end

%% LTI Tests
function testL2Lti(testCase)
    % Induced L2 norm and Hinf norm are equivalent for stable LTI systems
    sys = testCase.TestData.sys;
    lpvsys = testCase.TestData.lpvsys;

    l2LPV = lpvnorm(lpvsys,'l2','verbose',0);
    l2LTI = hinfnorm(sys);

    verifyLessThanOrEqual(testCase, abs(l2LPV-l2LTI), 10*sqrt(eps)); 
end

function testPassiveLti(testCase)
    sys = testCase.TestData.sys;
    lpvsys = testCase.TestData.lpvsys;

    passiveLTI = isPassive(sys);
    passiveLPV = lpvnorm(lpvsys,'passive','verbose',0);

    check = passiveLTI && passiveLPV;

    verifyTrue(testCase, check);  
end
 
function testH2Lti(testCase)
    % Generalized H2 norm is equivalent to H2 norm for siso systems, hence
    % why use sys1 here
    sys = testCase.TestData.sys1;
    lpvsys = testCase.TestData.lpvsys1;

    h2LPV = lpvnorm(lpvsys,'h2','verbose',0);
    h2LTI = norm(sys,2);

    verifyLessThanOrEqual(testCase, abs(h2LPV-h2LTI), sqrt(eps)); 
end

%% Grid and affine tests (grid should be (almost) equal to affine)
% L2
function testL2GridCT(testCase)
    lpvsysAffine = testCase.TestData.lpvsys2;
    lpvsysGrid = testCase.TestData.lpvsys3;

    [gam]      = lpvnorm(lpvsysAffine,'l2','verbose',0);
    [gam_grid] = lpvnorm(lpvsysGrid,'l2','verbose',0);

    verifyLessThanOrEqual(testCase, abs(gam-gam_grid), 1e-5)
end

function testL2GridDT(testCase)
    lpvsysAffine = testCase.TestData.lpvsys4;
    lpvsysGrid = testCase.TestData.lpvsys5;

    [gam]      = lpvnorm(lpvsysAffine,'l2','verbose',0);
    [gam_grid] = lpvnorm(lpvsysGrid,'l2','verbose',0);

    verifyLessThanOrEqual(testCase, abs(gam-gam_grid), 1e-5)
end

% H2
function testH2GridCT(testCase)
    lpvsysAffine = testCase.TestData.lpvsys2;
    lpvsysGrid = testCase.TestData.lpvsys3;

    lpvsysAffine.D = 0;
    lpvsysGrid.D = 0;

    [gam]      = lpvnorm(lpvsysAffine,'h2','verbose',0);
    [gam_grid] = lpvnorm(lpvsysGrid,'h2','verbose',0);

    verifyLessThanOrEqual(testCase, abs(gam-gam_grid), 1e-5)
end

function testH2GridDT(testCase)
    lpvsysAffine = testCase.TestData.lpvsys4;
    lpvsysGrid = testCase.TestData.lpvsys5;

    lpvsysAffine.D = 0;
    lpvsysGrid.D = 0;

    [gam]      = lpvnorm(lpvsysAffine,'h2','verbose',0);
    [gam_grid] = lpvnorm(lpvsysGrid,'h2','verbose',0);

    verifyLessThanOrEqual(testCase, abs(gam-gam_grid), 1e-5)
end

% Passive
function testPassiveGridCT(testCase)
    lpvsysAffine = testCase.TestData.lpvsys2;
    lpvsysGrid = testCase.TestData.lpvsys3;

    [pas]      = lpvnorm(lpvsysAffine,'passive','verbose',0);
    [pas_grid] = lpvnorm(lpvsysGrid,'passive','verbose',0);

    check = pas && pas_grid;
    verifyTrue(testCase, check)
end

function testPassiveGridDT(testCase)
    lpvsysAffine = testCase.TestData.lpvsys4;
    lpvsysGrid = testCase.TestData.lpvsys5;

    [pas]      = lpvnorm(lpvsysAffine,'passive','verbose',0);
    [pas_grid] = lpvnorm(lpvsysGrid,'passive','verbose',0);

    check = pas && pas_grid;
    verifyTrue(testCase, check)
end

% Linf
function testLinfGridCT(testCase)
    lpvsysAffine = testCase.TestData.lpvsys2;
    lpvsysGrid = testCase.TestData.lpvsys3;

    [gam]      = lpvnorm(lpvsysAffine,'linf','verbose',0);
    [gam_grid] = lpvnorm(lpvsysGrid,'linf','verbose',0);

    verifyLessThanOrEqual(testCase, abs(gam-gam_grid), 1e-5)
end

function testLinfGridDT(testCase)
    lpvsysAffine = testCase.TestData.lpvsys4;
    lpvsysGrid = testCase.TestData.lpvsys5;

    [gam]      = lpvnorm(lpvsysAffine,'linf','verbose',0);
    [gam_grid] = lpvnorm(lpvsysGrid,'linf','verbose',0);

    verifyLessThanOrEqual(testCase, abs(gam-gam_grid), 1e-5)
end