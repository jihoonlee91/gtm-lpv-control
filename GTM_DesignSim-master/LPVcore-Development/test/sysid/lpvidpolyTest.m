function tests = lpvidpolyTest
%LPVIDPOLYTEST Test behavior of LPVIDPOLY
    tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(testCase) % Called once before the tests in this file are run
    tm = timemap(randi(1, 1, 1) - 1, ...
        'dt', ...
        'Name', {'a', 'c'}, ...
        'Unit', {'-', 'm'}, ...
        'Range', {[-100, 100], [-Inf, +Inf]});

    testCase.TestData.P1 = pmatrix(rand(4, 4, 2) - 0.5, ...
        'BasisType', 'affine', ...
        'SchedulingTimeMap', tm);
    testCase.TestData.P2 = pmatrix(randn(4, 4, 2), ...
        'BasisType', 'affine', ...
        'SchedulingTimeMap', tm);
    
    testCase.TestData.sys = lpvidpoly({eye(4), testCase.TestData.P1}, ...
        {testCase.TestData.P2});
end

%% Test functions
function testCreation(testCase)
    d = preal('p', 'dt');
    c = preal('p', 'ct');
    
    sys = lpvidpoly({1, 1 + d});
    
    % Check sampling time, IO size
    verifyEqual(testCase, sys.Ny, 1);
    verifyEqual(testCase, sys.Nu, 0);
    verifyEqual(testCase, sys.Np, 1);
    verifyEqual(testCase, sys.Ts, -1);
    verifyEqual(testCase, lpvidpoly({1, c}).Ts, 0);
    
    % Filters must have compatible sizes
    verifyError(testCase, ...
        @() lpvidpoly(1, [], [1, 2]), '');
    verifyError(testCase, ...
        @() lpvidpoly(1, [1, 2], 1, 2, [1; 2]), '');
    verifyError(testCase, ...
        @() lpvidpoly(1, [1; 2]), '');
    
    % Filters must be monic
    verifyError(testCase, ...
        @() lpvidpoly(0.5), '');
    verifyError(testCase, ...
        @() lpvidpoly([1, 2; 0, 1]), '');
    verifyError(testCase, ...
        @() lpvidpoly({0.5}), '');
    verifyError(testCase, ...
        @() lpvidpoly({eye(3) * d}), '');
    verifyError(testCase, ...
        @() lpvidpoly(1, [], 0, 1, 1), '');
    verifyError(testCase, ...
        @() lpvidpoly(1, [], 1, 0, 1), '');
    verifyError(testCase, ...
        @() lpvidpoly(1, [], 1, 1, 0), '');
end

function testNParams(testCase)
    % Verify whether the number of (free) parameters is correct
    sys = lpvidpoly({1, 2, 3}, {1, 2, 3}, {1, 2, 3}, [], {1, 2, 3});
    syslti = idpoly([1, 2, 3], [1, 2, 3], [1, 2, 3], [], [1, 2, 3]);
    
    % Attempting to make a monic parameter free has no effect
    sys.A.Free = {1, 0, 0};
    syslti.Structure.A.Free = [1, 0, 0];
    
    verifyEqual(testCase, nparams(sys), nparams(syslti));
    verifyEqual(testCase, nparams(sys, 'free'), nparams(syslti, 'free'));

    % ZeroIsNonFree behavior
    sys = lpvidpoly({eye(2), [0, 1; 1, 0]}, {[1; 0]});
    verifyEqual(testCase, nparams(sys, 'free'), 3);
    verifyEqual(testCase, nparams(sys), 6);
    
    sys = lpvidpoly({eye(2), [0, 1; 1, 0]}, {[1; 0]}, 'ZeroIsNonFree', false);
    verifyEqual(testCase, nparams(sys, 'free'), 6);
    
    % Examples with scheduling variable
    p = preal('p', 'ct');
    sys = lpvidpoly({ eye(2), randn(2) * p + randn(2) });
    verifyEqual(testCase, nparams(sys), nparams(sys, 'free'));
    verifyEqual(testCase, nparams(sys), 8);
    % Setting all parameters to non-free
    sys.A.Free{2} = false(2, 2, 2);
    verifyEqual(testCase, nparams(sys), 8);
    verifyEqual(testCase, nparams(sys, 'free'), 0);
end

function testSize(testCase)
    sys = testCase.TestData.sys;
    P1 = testCase.TestData.P1;
    
    verifyTrue(testCase, all([size(P1, 1), size(P1, 2)] == size(sys)));
end

function testExtractLocal(testCase)
    sys = testCase.TestData.sys;
    verifyClass(testCase, extractLocal(sys, zeros(1, sys.Np)), 'idpoly');  
    sys.C = {eye(sys.Ny), randn(sys.Ny)};
    verifyError(testCase, @() extractLocal(sys, zeros(1, sys.Np)), '');
    % Compare lpvidpoly and idpoly for MIMO OE model
    % Discrete-time
    p = preal('p', 'dt');
    B = [1, 1; 1, 1] + p * randn(2);
    F = {eye(2), diag([2, 3]) + p * diag(randn(1, 2))};
    Blti = {1, 1; 1, 1};
    Flti = {[1, 2], [1, 2]; [1, 3], [1, 3]};
    sys = lpvidpoly([], B,    [], [], F);
    syslti = idpoly([], Blti, [], [], Flti);
    % Use extractLocal to go from lpvidpoly --> idpoly
    syslocal = extractLocal(sys, 0);
    verifyEqual(testCase, syslti, syslocal);
end

function testLSim(testCase)
    p = preal('p', 'dt');
    A = {1, p, pshift(p, -1)^2};
    B = {p + 0.5, p^3, p^2};
    C = {1, 0.6 - p, p^2};
    D = {1, 0.5};
    F = {1, p^2, p - 0.5};
    sys = lpvidpoly(A, B, C, D, F);
    
    N = 100;
    np = sys.Np;
    ny = sys.Ny;
    
    pFrozen = 0.1;
    pData = pFrozen * ones(N, np);
    uData = ones(N, np);
    
    [y, t] = lsim(sys, pData, uData, [], zeros(N, ny));
    sysLocal = extractLocal(sys, pFrozen);
    [yLocal, tLocal] = lsim(sysLocal, uData);
    
    verifyTrue(testCase, max(abs(y - yLocal)) < 2 * eps && ...
        max(abs(t - tLocal)) < 2 * eps);
    
    %% Trivial system
    syslpv = lpvidpoly(1, 1);
    syslpv.NoiseVariance = 0;
    syslti = idpoly(1, 1);
    
    N = 10;
    u = randn(N, 1);
    p = randn(N, 0);
    ylpv = lsim(syslpv, p, u);
    ylti = lsim(syslti, u);
    
    verifyTrue(testCase, norm(ylpv - ylti) < 1E-4);
end

function testInputDelay(testCase)
    syslpv = lpvidpoly(1, [1, 2], [], [], [], 0);
    syslti = idpoly(1, {1, 2}, [], [], [], 0);
    syslpv.InputDelay = [1, 3];
    syslti.InputDelay = [1, 3];
    
    N = 10;
    u = randn(N, syslpv.Nu);
    p = randn(N, syslpv.Np);
    
    ylpv = lsim(syslpv, p, u);
    ylti = lsim(syslti, u);
    
    verifyTrue(testCase, norm(ylpv - ylti) < 1E-4);
end

function testPredict(testCase)
    sys = testCase.TestData.sys;
    
    N = 200;        % nr. of samples
    K = 1;          % prediction horizon
    nu = sys.Nu;
    np = sys.Np;
    ny = sys.Ny;
    
    p = randn(N, np);
    u = randn(N, nu);
    [y, t, e] = lsim(sys, p, u, [], zeros(N, ny));
    yp = predict(sys, [y, p, u], K);
    
    verifyTrue(testCase, max(max(abs(yp - (y - e)))) <= 1E-10);
    verifyTrue(testCase, all(size(y) == [N, ny]) && ...
        numel(t) == N);
end
