function tests = lpvidssTest
%LPVIDSSTEST Test behavior of LPVIDSS
    tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(~) % Called once before the tests in this file are run
    rng(1);
end

%% Test functions
function testLSim(testCase)
    N = 10;

    A = 0.9;
    B = 1.2;
    C = -1;
    D = 1.5;

    sys = lpvidss(A, B, C, D, 'innovation');
    sys_lti = idss(A, B, C, D);

    sys.NoiseVariance = 0;
    sys_lti.NoiseVariance = 0;

    u = randn(N, sys.Nu);
    p = zeros(N, sys.Np);
    x0 = randn(sys.Nx);

    [y, t, w, yp] = lsim(sys, p, u, [], x0);
    y_lti = lsim(sys_lti, u, [], x0);

    verifyTrue(testCase, norm(y - y_lti) < 1E-10);
    % y and yp should be equal if no noise is present
    verifyEqual(testCase, y, yp);
    % t should have spacing 1 if sampling time is not specified
    verifyEqual(testCase, t, (0:N-1).');
    % noise is 0 since NoiseVariance was set to 0
    verifyEqual(testCase, norm(w), 0);
end

function testPredict(testCase)
    N = 10;

    A = 0.1 * randn(2);
    B = randn(2, 1);
    C = randn(1, 2);
    D = randn;

    sys = lpvidss(A, B, C, D, 'innovation');
    sys_lti = idss(A, B, C, D);

    sys.NoiseVariance = 0;
    sys_lti.NoiseVariance = 0;

    u = randn(N, sys.Nu);
    p = zeros(N, sys.Np);
    x0 = randn(sys.Nx, 1);

    y = lsim(sys, p, u, [], x0);
    y_lti = lsim(sys_lti, u, [], x0);

    data = lpviddata(y, p, u);
    data_lti = iddata(y_lti, u);

    opt = predictOptions('InitialCondition','z');

    y_pred = predict(sys, data, 1);
    y_lti_pred = predict(sys_lti, data_lti, 1, opt);
    
    verifyTrue(testCase, norm(y_pred - y_lti_pred.y) < 1E-10);
end
