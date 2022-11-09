function tests = bjTest
%BJTEST Test identification using LPVBJ
    tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(testCase) % Called once before the tests in this file are run
    rng(42);   % set random number generator seed so that results are reproducible

    p = preal('p', 'dt');
    B = {1, 0.5 * p};
    F = {1, -0.1 * p};
    C = {1, 0.2 * p + p^2};
    D = {1, 0.4 * p^2};
    sys = lpvidpoly([], B, C, D, F);
    sys.NoiseVariance = 1E-3;
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
function testBj(testCase)
    true_sys = testCase.TestData.sys;
    data = testCase.TestData.data;
    val_data = testCase.TestData.val_data;
    template_sys = randlpvidpoly(true_sys);
    % Gradient-based
    lpvbj(data, template_sys, ...
        lpvbjOptions('Verbose', true, 'Regularization', struct('Lambda', 1E-5)));
    % Pseudo-LS
    est_sys = lpvbj(data, template_sys, ...
        lpvbjOptions('SearchMethod', 'pseudols'));
    % Compute fit w.r.t. validation data
    [~, fit] = compare(val_data, est_sys, 1);
    fprintf('Fit LPV-BJ: %.2f%%\n', fit);
    verifyEqual(testCase, fit, 95.81, 'AbsTol', 0.01);
end
