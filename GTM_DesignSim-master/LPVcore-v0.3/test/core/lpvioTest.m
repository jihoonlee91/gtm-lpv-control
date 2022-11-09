function tests = lpvioTest
%LPVIOTEST Test behavior of LPVIO
    tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(testCase) % Called once before the tests in this file are run
rng(2)
    tm1 = timemap(randi(201, 1, 10) - 101, ...
        'dt', ...
        'Name', {'a', 'b'}, ...
        'Unit', {'-', 'cm'}, ...
        'Range', {[-100, 100], [0, 1]});
    tm2 = timemap(randi(201, 1, 10) - 101, ...
        'dt', ...
        'Name', {'a', 'c'}, ...
        'Unit', {'-', 'm'}, ...
        'Range', {[-100, 100], [-Inf, +Inf]});

    testCase.TestData.P1 = pmatrix(randn(4, 4, 2), ...
        'BasisType', 'affine', ...
        'SchedulingTimeMap', tm1);
    testCase.TestData.P2 = pmatrix(randn(4, 4, 2), ...
        'BasisType', 'affine', ...
        'SchedulingTimeMap', tm2);

    testCase.TestData.sys = lpvio({eye(4), testCase.TestData.P1}, {testCase.TestData.P2}, 0.1);
end

%% Test functions
function testCreation(testCase)
    % Test illegal creation
    A = {1, randn(2)};  % incompatible sizes
    verifyError(testCase, ...
        @() lpvidpoly(A), '');
    % Default sampling time
    A = {1, preal('p', 'dt')};
    sys = lpvio(A, {1});
    verifyEqual(testCase, sys.Ts, -1);
    sys = lpvio({1}, {preal('p', 'ct')});
    verifyEqual(testCase, sys.Ts, 0);
    % No input
    sys = lpvio({eye(2)}, {});
    verifyEqual(testCase, sys.Ny, 2);
    verifyEqual(testCase, sys.Nu, 0);
    verifyEqual(testCase, sys.Np, 0);
end

function testDisp(testCase)
    disp(testCase.TestData.sys);
end

function testSize(testCase)
    sys = testCase.TestData.sys;
    P1 = testCase.TestData.P1;
    
    verifyEqual(testCase, [size(P1, 1), size(P1, 2)], size(sys));
end

function testLSim(testCase)
    sys = testCase.TestData.sys;
    
    N = 100;
    nu = sys.Nu;
    np = sys.Np;
    ny = sys.Ny;
    
    [y, t] = lsim(sys, randn(N, np), randn(N, nu));
    verifySize(testCase, y, [N, ny]);
    verifySize(testCase, t, [N, 1]);
end

function testInputDelay(testCase)
    % Test setting
    syslpv = lpvio(1, [1, 1]);
    syslpv.InputDelay = 1;
    syslpv.InputDelay = [1, 1];

    % Test simulation
    syslpv = lpvio(1, 1);
    verifyEqual(testCase, syslpv.Np, 0);
    syslti = ss(1);
    
    syslpv.InputDelay = 3;
    syslti.InputDelay = 3;
    
    N = 10;
    u = randn(N, syslpv.Nu);
    p = randn(N, syslpv.Np);
    
    ylpv = lsim(syslpv, p, u);
    ylti = lsim(syslti, u, 0:N-1);
    
    verifyEqual(testCase, ylpv, ylti, 'AbsTol', 1E-4);
end

function testABSet(testCase)
    sys = testCase.TestData.sys;
    A = randn(sys.Ny);
    B = randn(sys.Ny, sys.Nu);
    sys.A = {eye(sys.Ny), A};
    sys.B = B;
    verifyEqual(testCase, sys.Na, 1);
    verifyEqual(testCase, sys.Nb, 0);
    verifyTrue(testCase, sys.A{2} == A);
    verifyTrue(testCase, sys.B{1} == B);
end

function testCtSim(testCase)
    % create model
    p = preal('p', 'ct');
    n = 2;
    sys = lpvio({eye(n), randn(n), 2 * eye(n) + p * zeros(n)}, ...
        {ones(n) + p * ones(n)});
    
    pFrozen = ones(1,sys.Np);
    sysLocal = extractLocal(sys, pFrozen);
    
    nu = sys.Nu;
    N = 10;
    uData = 2 * ones(N, nu);
    pData = ones(N, sys.Np);
    t = linspace(0, .1, N)';
    
    % simulate using lpvio 
    y = lsim(sys, pData,uData, t);
    yLocal = lsim(sysLocal, uData, t, 0, 'foh');
    
    verifyEqual(testCase, y, yLocal, 'AbsTol', 0.01);
    
    %% Trivial system y = u
    sys = lpvio({eye(4)}, {eye(4) * p});
    p = ones(N, sys.Np);
    u = randn(N, sys.Nu);
    t = linspace(0, 1, N).';
    y = lsim(sys, p, u, t);
    verifyEqual(testCase, y, u, 'AbsTol', 1E-10);
end

function testDtSim(testCase)
    %% Trivial system y = u (even without p-matrices it is simulated using LPV-io)
    %simplest trivial (SISO-)system
    p = preal('p', 'dt');    
    N = 10;
    sys = lpvio({eye(1)}, {eye(1)});
    p1 = ones(N, sys.Np);
    u = randn(N, sys.Nu);
    y = lsim(sys, p1, u);
    verifyEqual(testCase, y, u, 'AbsTol', 1E-10);
    %slightly less trivial SISO 
    sys = lpvio({eye(1)}, {eye(1)*2*p});
    p1 = ones(N, sys.Np);
    u = randn(N, sys.Nu);
    y = lsim(sys, p1, u);
    verifyEqual(testCase, y, 2*u, 'AbsTol', 1E-10);
    %larger trivial system
    sys = lpvio({eye(4)}, {eye(4)});
    p2 = ones(N, sys.Np);
    u = randn(N, sys.Nu);
    y = lsim(sys, p2, u);
    verifyEqual(testCase, y, u, 'AbsTol', 1E-10);

    % More Complex 
    shiftAmount = -3; %as long as this is negative use 'p0' in lsym, otherwise use 'pf'
    q = pshift(p,shiftAmount);
    % More complex SISO:
    ny = 1; nu = 1;
    sys = lpvio({eye(ny), randn(ny)}, ...
        {ones(ny,nu) + p * ones(ny,nu), randn(ny,nu)+q*eye(ny,nu), p*randn(ny,nu)});
    pFrozen = ones(1,sys.Np);
    sysLocal = extractLocal(sys, pFrozen);
    nu = sys.Nu;
    uData = 2 * ones(N, nu);
    pData = pFrozen*ones(N, 1);
    y  = lsim(sys, pData ,uData, 'p0', pFrozen*ones(abs(shiftAmount), 1));
    yLocal = lsim(sysLocal, uData);
    verifyEqual(testCase, y, yLocal, 'RelTol', 1E-10);
    
    % More Complex MIMO
    ny = 2; nu = 3;
    sys = lpvio({eye(ny), randn(ny)}, ...
        {ones(ny,nu) + p * ones(ny,nu), randn(ny,nu)+q*eye(ny,nu), p*randn(ny,nu)});
    pFrozen = ones(1,sys.Np);
    sysLocal = extractLocal(sys, pFrozen);
    uData = 2 * ones(N, nu);
    pData = pFrozen*ones(N, 1);
    % simulate using lpvio 
    y  = lsim(sys, pData ,uData, 'p0', pFrozen*ones(abs(shiftAmount), 1));
    yLocal = lsim(sysLocal, uData);
    verifyEqual(testCase, y, yLocal, 'RelTol', 1E-10);
    
    % Trivial initial conditions MIMO
    sys = lpvio({eye(ny), -0.5*eye(ny)}, {ones(ny,nu)});
    p1 = ones(N, sys.Np);
    u = zeros(N, sys.Nu);
    y0 = ones(1, ny)*2^N;
    y = lsim(sys, p1, u, 'y0', y0);
    yGT = ones(1, ny).*2.^(N-1:-1:0)' ;
    verifyEqual(testCase, y, yGT, 'AbsTol', 1E-10);
    
    sys = lpvio({eye(ny)}, {ones(ny,nu), zeros(ny,nu) , 0.5*ones(ny,nu)});
    p1 = ones(N, sys.Np);
    u = ones(N, sys.Nu);
    u0 = [10 8]'.*ones(1,sys.Nu);
    y = lsim(sys, p1, u, 'u0', u0);
    yGT = (u(:,1) + [5 4 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5]').*ones(1, ny)*nu;
    verifyEqual(testCase, y, yGT, 'RelTol', 1E-10);
    
   % Trivial InputDelay MIMO
    sys = lpvio({eye(ny)}, {ones(ny,nu), zeros(ny,nu) , ones(ny,nu)*0.5});
    delay = 3;
    sys.InputDelay = delay;
    p1 = ones(N, sys.Np);
    u = ones(N, sys.Nu);
    u0 = [10 8]'*ones(1,nu);
    y = lsim(sys, p1, u, 'u0', u0);
    % next 3lines are [u0', u'] with 'delay'-size(u0,2) leading 0 
    %  u =   0 10  8  1  1  1   1   1   1   1  
    %0 u-1 = 0 0   0  0  0  0   0   0   0   0 
    %.5 u-2= 0 0   0  5  4 .5  .5  .5  .5  .5 
    yGT = [  0 10  8  6  5 1.5 1.5 1.5 1.5 1.5]'*nu*ones(1,ny);
    verifyEqual(testCase, y, yGT, 'RelTol', 1E-10);
    
    % More Complex MIMO
    ny = 2; nu = 3;
    sys = lpvio({eye(ny), randn(ny)}, ...
        {ones(ny,nu) + p * ones(ny,nu), randn(ny,nu)+q*eye(ny,nu), p*randn(ny,nu)});
    sys.InputDelay = 3;
    pFrozen = ones(1,sys.Np);
    sysLocal = extractLocal(sys, pFrozen);
    uData = 2 * ones(N, nu);
    pData = pFrozen*ones(N, 1);
    % simulate using lpvio 
    y  = lsim(sys, pData ,uData, 'p0', pFrozen*ones(abs(shiftAmount), 1));
    yLocal = lsim(sysLocal, uData);
    verifyEqual(testCase, y, yLocal, 'RelTol', 1E-10);
end

function testProperties(testCase)
    inputDelay = [1; 1];
    inputName = {'u1', 'u2'};
    outputName = {'y1', 'y2'};
    inputUnit = {'cm', 'm'};
    outputUnit = {'m/s', 'J'};
    nu = numel(inputName);
    ny = numel(outputName);
    p = preal('p', 'ct');
    Ts = 0;
    A = {eye(ny), p * eye(ny)};
    B = {randn(ny, nu)};
    sys = lpvio(A, B, Ts, ...
        'InputDelay', inputDelay, ...
        'InputName', inputName, ...
        'OutputName', outputName, ...
        'InputUnit', inputUnit, ...
        'OutputUnit', outputUnit);
    syslti = ss(1, ones(1, nu), ...
        ones(ny, 1), ones(ny, nu), Ts, ...
        'InputDelay', inputDelay, ...
        'InputName', inputName, ...
        'OutputName', outputName, ...
        'InputUnit', inputUnit, ...
        'OutputUnit', outputUnit);
    
    verifyTrue(testCase, areSharedPropsEqual(sys, syslti))
end

