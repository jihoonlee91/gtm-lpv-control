function tests = bayesopsTest
%OETEST Test operating point selection using BAYESOPS
    tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(~) % Called once before the tests in this file are run
end

%% Test functions
function testBayesopsCT(testCase)
    rng(1);
    % CT time
    G1 = rss(2, 1, 1);
    G2 = rss(2, 1, 1);
    G = {G1, G2};
    p1 = [-1, -1];
    p2 = [1, 1];
    p = {p1, p2};
    
    [pNext, nugapEst, results] = bayesops(G, p);
    
    verifySize(testCase, pNext, size(p{1}));
    verifyInstanceOf(testCase, nugapEst, 'function_handle');
    verifyEqual(testCase, nugapEst(p1, p1), 0, 'AbsTol', 1E-4);
    verifyEqual(testCase, nugapEst(p2, p2), 0, 'AbsTol', 1E-4);
    verifyInstanceOf(testCase, results, 'BayesianOptimization');
end

function testBayesopsDT(testCase)
    rng(2);
    % DT time and SISO
    G1 = drss(3, 2, 2);
    G2 = drss(3, 2, 2);
    G = {G1, G2};
    p1 = 1;
    p2 = 2;
    p = {p1, p2};
    
    [pNext, nugapEst, results] = bayesops(G, p);
    
    verifySize(testCase, pNext, size(p{1}));
    verifyInstanceOf(testCase, nugapEst, 'function_handle');
    verifyEqual(testCase, nugapEst(p1, p1), 0, 'AbsTol', 1E-4);
    verifyEqual(testCase, nugapEst(p2, p2), 0, 'AbsTol', 1E-4);
    verifyInstanceOf(testCase, results, 'BayesianOptimization');
end