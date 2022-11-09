function tests = compareTest
%COMPARETEST Test compare function
    tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(testCase) % Called once before the tests in this file are run
    rng(42);   % set random number generator seed so that results are reproducible

    p = preal('p', 'dt');
    B = {1, 0.5 * p};
    F = {1, 0.5 * p};
    sys = lpvidpoly([], B, [], [], F);
    sys.NoiseVariance = 1E-10;
    N = 100;
    u = randn(N, sys.Nu);
    p = rand(N, sys.Np);
    y = lsim(sys, p, u);
    testCase.TestData.sys = sys;
    testCase.TestData.data = lpviddata(y, p, u);
    u = randn(N, sys.Nu);
    p = rand(N, sys.Np);
    y = lsim(sys, p, u);
    testCase.TestData.val_data = lpviddata(y, p, u);
end

%% Test functions
function testOe(testCase)
    true_sys = testCase.TestData.sys;
    val_data = testCase.TestData.val_data;
    [ySim, fit] = compare(val_data, true_sys, 1);
    verifyTrue(testCase, norm(ySim - val_data.y) < 1E-3, ...
        'Compare did not return valid output.');
    verifyTrue(testCase, fit >= 0.999, ...
        'Compare did not return perfect fit for true system.');
    % Check argument combinations and plotting
    compare(val_data, true_sys, 1);
    compare(val_data, true_sys, 'g', 1);
    compare(val_data, true_sys, 'g');
    close all;
end

function testSyntax(testCase)
    % Test syntax of compare for different input arguments
    true_sys = testCase.TestData.sys;
    data = testCase.TestData.val_data;
    id_sys = lpvidss(preal('p', 'dt'), 1, 1, 1, 'innovation');
    id_sys_io = lpvidpoly(1, {1, preal('p', 'dt')});
    id_sys.Ts = true_sys.Ts;
    id_sys_io.Ts = true_sys.Ts;

    % Check different calling syntaxes
    compare(data, id_sys, 'g', 1);
    compare(data, id_sys, 1);
    compare(data, id_sys, Inf);
    compare(data, id_sys, id_sys, id_sys, Inf);
    compare(data, id_sys, id_sys, 'k--', id_sys, Inf);
    compare(data, id_sys, 'g', id_sys, 'k--', id_sys, '--');
    verifyError(testCase, @() compare(id_sys), 'MATLAB:narginchk:notEnoughInputs');
    verifyError(testCase, @() compare(data, data), '');
    verifyError(testCase, @() compare(data, 'g', id_sys, 1), '');
    verifyError(testCase, @() compare(data, id_sys, 2), '');  % invalid k
    % Check output arguments
    [ySys, fit] = compare(data, id_sys, id_sys, Inf);
    verifyEqual(testCase, fit{1}, fit{2});
    verifyEqual(testCase, ySys{1}, ySys{2});
    [ySys, ~] = compare(data, id_sys, id_sys, 1);
    verifyEqual(testCase, ySys{1}, ySys{2});
    % Test with LPVIDPOLY
    [ySys, ~] = compare(data, id_sys_io, id_sys_io, Inf);
    verifyEqual(testCase, ySys{1}, ySys{2});
    [ySys, ~] = compare(data, id_sys_io, id_sys_io, 1);
    verifyEqual(testCase, ySys{1}, ySys{2});
    % Only DT models
    verifyError(testCase, ...
        @() compare(data, lpvidss(preal('p', 'ct'), 1, 1, 1, 'innovation')), '');
    close all
end
