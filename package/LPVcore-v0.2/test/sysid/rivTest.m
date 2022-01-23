function tests = rivTest
%RIVTEST Test identification using RIV
    tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(~) % Called once before the tests in this file are run
    rng(1);
end

%% Test functions
function testRiv(testCase)
    Ts = 0.1;  % s
    NoiseVariance = 1E-1;

    p = preal('p', 'dt');

    B = {1+p, 1+p};
    F = {1, 1+p};

    sys = lpvidpoly([], B, {1, 1}, {1, 1}, F, NoiseVariance, Ts);

    %% Data generation
    N = 100;  % samples

    % Train
    uData = randn(N, sys.Nu);
    pData = randn(N, sys.Np);
    tData = Ts * (1:N);
    [y, ~, ~, yp] = lsim(sys, pData, uData, tData);
    fprintf('SNR is %.1f dB\n', LPVcore.snr(y, y - yp));

    data_est = lpviddata(y, pData, uData, Ts);

    %% Identification
    template_sys = sys;

    sys_est = riv(data_est, template_sys);

    [~, fit] = compare(data_est, sys_est);
    fprintf('Fit is %.1f%%\n', fit);
    verifyEqual(testCase, fit, 96.3, 'AbsTol', 1);
    
    %% Invalid invocations
    % Too few arguments
    verifyError(testCase, @() riv(data_est), 'MATLAB:narginchk:notEnoughInputs');
    % Unequal parametrizations
    template_sys = lpvidpoly([], {1, p}, {1, 1}, {1, 1}, {1, p}, NoiseVariance, Ts);
    verifyError(testCase, @() riv(data_est, template_sys), '');
    % Non-constant noise process
    template_sys = lpvidpoly([], B, {1, p}, {1, 1}, F, NoiseVariance, Ts);
    verifyError(testCase, @() riv(data_est, template_sys), '');
    % Continuous-time system
    template_sys = lpvidpoly([], {preal('p', 'ct')}, {1, 1}, {1, 1}, {1, preal('p', 'ct')}, NoiseVariance, 0);
    verifyError(testCase, @() riv(data_est, template_sys), '');
    % MIMO system
    template_sys = lpvidpoly([], {preal('p', 'dt') * ones(1, 2)}, [], [], [], NoiseVariance, Ts);
    verifyError(testCase, @() riv(data_est, template_sys), '');
end
