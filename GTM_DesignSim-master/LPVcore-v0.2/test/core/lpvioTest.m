function tests = lpvioTest
%LPVIOTEST Test behavior of LPVIO
    tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(testCase) % Called once before the tests in this file are run
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
    % create model
    p = preal('p', 'dt');
    n = 2;
    sys = lpvio({eye(n), randn(n), 2 * eye(n) + p * zeros(n)}, ...
        {ones(n) + p * ones(n), randn(n)});
    
    pFrozen = ones(1,sys.Np);
    sysLocal = extractLocal(sys, pFrozen);
    
    nu = sys.Nu;
    N = 10;
    uData = 2 * ones(N, nu);
    pData = ones(N, sys.Np);
    
    % simulate using lpvio 
    y = lsim(sys, pData,uData);
    yLocal = lsim(sysLocal, uData);
    
    verifyEqual(testCase, y, yLocal, 'AbsTol', 1E-10);
    
    %% Trivial system y = u
    sys = lpvio({eye(4)}, {eye(4) * p});
    p = ones(N, sys.Np);
    u = randn(N, sys.Nu);
    y = lsim(sys, p, u);
    verifyEqual(testCase, y, u, 'AbsTol', 1E-10);
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

