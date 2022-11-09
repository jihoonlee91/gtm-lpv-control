function tests = lpvrepbodeTest
    %LPVLFR Test behavior of LPVLFR
    tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(testCase) % Called once before the tests in this file are run
    G1 = drss(3, 3, 4);
    G2 = drss(3, 3, 4);
    p1 = preal('p1', 'dt');
    p2 = preal('p2', 'dt');
    Delta = p1 * randn(2) + p2 * rand(2);
    testCase.TestData.sys1 = lpvlfr(Delta, G1);
    testCase.TestData.sys2 = lpvlfr(Delta, G2);
end

function testbode(testCase)
    close all;
    sys1 = testCase.TestData.sys1;
    sys2 = testCase.TestData.sys2;
    rho = 0.1:0.5:1;
    rho = rho';
    % test1
    figure
    bode(sys1, sys2, [rho rho])

    % test2
    figure
    bode(sys1,'r', sys2,'b*', [rho rho])

    % test 3
    figure
    w = logspace(-4,0,100);
    bode(sys1,sys2, [rho rho],w)
    verifyTrue(testCase, ishandle(3));
end

function testfreqresp(testCase)
    sys = testCase.TestData.sys1;
    w = 0.01:0.01:1;
    rho = randn(numel(w), sys.Np);
    % test 1
    out = freqresp(sys,rho);   
    % test 2
    [out1, wout1]= freqresp(sys,w,rho);
    % test 3
    [out2, wout2] = freqresp(sys,w,'rad/s',rho);
    % test 1    
    verifyTrue(testCase,length(out) == length(rho));
    % test 2
    verifyTrue(testCase,length(out1) == length(rho) && all(abs(w-wout1)<1e-6));
    % test 3
    verifyTrue(testCase,length(out2) == length(rho) && all(abs(w-wout2)<1e-6));
end

function testbodemag(testCase)
    close all;
    sys1 = testCase.TestData.sys1;
    sys2 = testCase.TestData.sys2;
    rho = 0.1:0.5:1;
    rho = rho';
    % test1
    figure
    bodemag(sys1, sys2, [rho rho])

    % test2
    figure
    bodemag(sys1,'r', sys2,'b*', [rho rho])

    % test 3
    figure
    w = logspace(-4,0,100);
    bodemag(sys1,sys2, [rho rho],w)
    verifyTrue(testCase, ishandle(3));
end

function testsigma(testCase)
    close all;
    sys1 = testCase.TestData.sys1;
    sys2 = testCase.TestData.sys2;
    rho = 0.1:0.5:1;
    rho = rho';
    % test1
    figure
    sigma(sys1, sys2, [rho rho])

    % test2
    figure
    sigma(sys1,'r', sys2,'b*', [rho rho])

    % test 3
    figure
    w = logspace(-4,0,100);
    sigma(sys1,sys2, [rho rho],w)
    verifyTrue(testCase, ishandle(3));
end
