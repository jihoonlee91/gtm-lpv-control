function tests = lpvssestTest
%LPVSSESTTEST Test identification using LPVSSEST
    tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(~) % Called once before the tests in this file are run
end

%% Test functions
function testEGN(testCase)
    %% Enhanced Gauss-Newton search method
    rng(42);
    %% Model structure
    p = preal('p', 'dt');
    nx = 2;
    nu = 1;
    ny = 1;

    A = p * 0.1 * randn(nx);
    B = randn(nx, nu) * p^2 + randn(nx, nu);
    C = randn(ny, nx);
    D = zeros(ny, nu);
    K = randn(nx, ny);
    
    Ainit = 0.98 * A;
    Binit = 1.02 * B;
    Cinit = 0.97 * C;
    Dinit = D;
    Kinit = 0.79 * K;
    
    NoiseVariance = 0.1;
    Ts = 0.1;
    
    sys = lpvidss(A, B, C, D, 'innovation', K, [], ...
        NoiseVariance, Ts);
    sys_init = lpvidss(Ainit, Binit, Cinit, Dinit, 'innovation', Kinit, [], ...
        NoiseVariance, Ts);

    %% Estimation

    N = 100;
    u = randn(N, nu);
    p = -0.5 + rand(N, sys.Np);

    y = lsim(sys, p, u);
    data = lpviddata(y, p, u, Ts);
    options = lpvssestOptions('Display', 'on');
    searchOptions = options.SearchOptions;
    searchOptions.maxIter = 10;
    options = lpvssestOptions('Display', 'on', 'SearchOptions', searchOptions);
    sys_est = lpvssest(data, sys_init, options);

    %% Validation
    u_val = randn(N, nu);
    p_val = -0.5 + rand(N, sys.Np);
    y_val = lsim(sys, p_val, u_val, [], zeros(nx, 1), zeros(size(u)));
    y_init = lsim(sys_init, p_val, u_val, [], zeros(nx, 1), zeros(size(u)));
    y_est = lsim(sys_est, p_val, u_val, [], zeros(nx, 1), zeros(size(u)));

    fit_init = LPVcore.goodnessOfFit(y_init, y_val, 'nmse');
    fit_est = LPVcore.goodnessOfFit(y_est, y_val, 'nmse');
    fprintf('Validation fit (init --> est): %.2f --> %.2f\n', fit_init, fit_est);
    
    verifyTrue(testCase, fit_est <= 1-0.965);

end

