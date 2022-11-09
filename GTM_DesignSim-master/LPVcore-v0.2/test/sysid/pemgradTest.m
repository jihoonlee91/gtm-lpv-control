function tests = pemgradTest
%PEMGRADTEST Test identification using PEM-GRADIENT
    tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(~) % Called once before the tests in this file are run
end

%% Test functions
function testPemGrad(testCase)
    rng(42);
    %% Model structure
    p = preal('p', 'dt');

    B = {1 + p, 4 + 2 * p * p};
    F = {1, 0.5 + 0.5 * p};

    C = {1, 1 + p};
    D = {1, 0.5 + 0.5 * p};

    B_init = {1 + p, 2 + 2 * p * p};
    F_init = {1, 0.4 + 0.4 * p};

    C_init = {1, 0.6 + 0.4 * p};
    D_init = {1, 0.4 + 0.6 * p};

    sys = lpvidpoly([], B, C, D, F);

    %% Estimation
    sys_init = lpvidpoly([], B_init, C_init, D_init, F_init);

    N = 100;
    u = randn(N, 1);
    p = -0.5 + rand(N, 1);

    y = lsim(sys, p, u);
    data = lpviddata(y, p, u);
    opts = lpvpolyestOptions('SearchMethod', 'gradient', ...
        'SearchOptions', ...
            struct('StepSize', 0.1, 'MaxIterations', 20), ...
        'Verbose', false, ...
        'Regularization', struct('Lambda', 1));
    sys_est = lpvpolyest(data, sys_init, opts);

    %% Validation
    u_val = randn(N, 1);
    p_val = -0.5 + rand(N, 1);
    y_val = lsim(sys, p_val, u_val, [], zeros(size(u)));
    y_init = lsim(sys_init, p_val, u_val, [], zeros(size(u)));
    y_est = lsim(sys_est, p_val, u_val, [], zeros(size(u)));

    fit_init = LPVcore.goodnessOfFit(y_init, y_val, 'nmse');
    fit_est = LPVcore.goodnessOfFit(y_est, y_val, 'nmse');
    fprintf('Validation fit (init --> est): %.2f --> %.2f\n', fit_init, fit_est);
    
    verifyTrue(testCase, fit_est <= 1-0.965);

end
