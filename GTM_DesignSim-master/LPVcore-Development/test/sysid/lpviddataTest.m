function tests = lpviddataTest
%LPVIDDATATEST Test behavior of LPVIDDATA
    tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(~) % Called once before the tests in this file are run
end

%% Test functions
function testCreate(testCase)
    N = 100;
    ny = 3;
    nu = 5;
    np = 4;
    y = randn(N, ny);
    p = randn(N, np);
    u = randn(N, nu);
    
    lpviddata(y, p, u);
    lpviddata([], p, u);
    lpviddata(y, [], u);
    lpviddata(y, p, []);
    lpviddata(y, [], []);
    verifyError(testCase, @() lpviddata([], [], []), ...
        'LPVcore:general:InvalidInput');
end


