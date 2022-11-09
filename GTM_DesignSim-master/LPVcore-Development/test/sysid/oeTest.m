function tests = oeTest
%OETEST Test identification using LPVOE
    tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(testCase) % Called once before the tests in this file are run
    rng(42);   % set random number generator seed so that results are reproducible

    p = preal('p', 'dt');
    B = {1, 0.5 * p};
    F = {1, 0.5 * p};
    sys = lpvidpoly([], B, [], [], F);
    sys.NoiseVariance = 1E-4;
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
    data = testCase.TestData.data;
    val_data = testCase.TestData.val_data;
    template_sys = randlpvidpoly(true_sys);
    % Regularization is required here
    est_sys = lpvoe(data, template_sys, ...
        lpvoeOptions('SearchMethod', 'pseudols'));
    % Compute fit w.r.t. validation data
    [~, fit] = compare(val_data, est_sys, 1);
    fprintf('Fit LPV-OE: %.2f%%\n', fit);
    verifyTrue(testCase, abs(fit - 98) <= 1);
end

function testInputDelay(testCase)
    % Verify whether Input Delay works correctly by comparing with OE.
    ts = 1;
    noiseVariance = 0.05;
    inputDelay = 2;
    p = preal('p', 'dt');

    sys = lpvidpoly({}, {1, p}, {}, {}, {1, 1}, noiseVariance, ts, ...
        'InputDelay', inputDelay);
    syslti = idpoly([], [1, 1], [], [], [1, 1], noiseVariance, ts, ...
        'InputDelay', inputDelay);

    N = 100;
    rng(1);
    u = randn(N, sys.Nu);
    p = ones(N, sys.Np);  % LTI
    y = lsim(sys, p, u);

    data = lpviddata(y, p, u);
    data_lti = iddata(y, u);

    template_sys = randlpvidpoly(sys);

    opts = lpvoeOptions('SearchMethod', 'pseudols');
    sys_est = lpvoe(data, template_sys, opts);
    sys_est_lti = oe(data_lti, [syslti.nb, syslti.nf, syslti.nk], ...
        'InputDelay', inputDelay);

    [~, fit] = compare(data, sys_est);
    [~, fit_lti] = compare(data_lti, sys_est_lti);
    
    % Fit should be within 1% of each other
    verifyEqual(testCase, fit, fit_lti, 'AbsTol', 1);
end
