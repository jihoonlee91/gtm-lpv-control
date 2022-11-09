function tests = pbasisTest
%PBASISTEST Test behavior of PBASIS objects
tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(~) % Called once before the tests in this file are run
end

%% Test functions
function testPBCustom(testCase)
    % Check creation
    verifyError(testCase, @() pbcustom('h'), '');
    verifyError(testCase, @() pbcustom(preal('p', 'dt')), '');
    % "rho" must be variable name
    verifyError(testCase, @() pbcustom(@(x) x), '');
    % "rho" must be vectorized
    verifyError(testCase, @() pbcustom(@(rho) rho), '');
    
    % String representation
    f = pbcustom(@(rho) rho(:, 1) + rho(:, 2));
    verifyEqual(testCase, str(f, {'p'}), 'p+rho(:,2)');
    verifyEqual(testCase, str(f, {'p', 'q'}), 'p+q');
    
    % Re-indexing
    f = pbcustom(@(rho) sin(rho(:, 1)) + cos(rho(:, 2)));
    fr = pbcustom(@(rho) sin(rho(:, 2)) + cos(rho(:, 1)));
    f = reidx(f, [2; 1]);
    verifyEqual(testCase, f.sortKey, fr.sortKey);
    
    % Multiplication
    f1 = pbcustom(@(rho) rho(:, 1));
    f2 = pbcustom(@(rho) rho(:, 2));
    f3 = f1 * f2;
    verifyEqual(testCase, str(f3), '(rho(:,1))*(rho(:,2))');
    % Swapping the index twice returns f1*f2 again
    f3 = reidx(f3, [2; 1]);
    f3 = reidx(f3, [2; 1]);
    verifyEqual(testCase, str(f3), '(rho(:,1))*(rho(:,2))');
end

function testPBinterp(testCase)
    %test for tentfunction (default)
    pb = pbinterp({[-1 1]}, [0 1]);
    rho = [0 0.5 1]';
    verifyEqual(testCase,feval(pb,rho), [ 0.5 0.75 1]', 'AbsTol', 1e-15)
    
    %test for reindexing
    pb = pbinterp({[-1 1], [0 1 2 6 7]}, [1 0 0 0 0; 0 0 1 0 0]);
    rho = [1 2; 1 3; 1 5; 0 6; -1 6.5; -.5 5];
    verifyEqual(testCase,feval(pb,rho), feval(reidx(pb, [2 1]), [rho(:,2), rho(:,1)]))
    verifyEqual(testCase,pb, reidx(reidx(pb, [2 1]), [2 1])) %swapping twice should result in the original
end

function testMinNRho(testCase)
    %normaly it is writen as verifyEqual(.., Actual, Expected, ...)
    %however, in this case verifyEqual(.., Expected, Actual, ...) gives
    %better readability.
    
    %Constants should have 0 required inputs to evaluate
    verifyEqual(testCase, 0, minNRho(pbconst()))
    verifyEqual(testCase, 0, minNRho(pbcustom(@(rho) 1)))
    
    %poly:
    verifyEqual(testCase, 3, minNRho(pbaffine(3)))
    verifyEqual(testCase, 4, minNRho(pbpoly([1 0 0 0; 0 0 2 0])))
    a = zeros(100);
    a([1:5 16 18 30], [1:2 5 20 50]) = 1;
    verifyEqual(testCase, 100, minNRho(pbpoly(a)))
    
    %custom:
    verifyEqual(testCase, 1,    minNRho(pbcustom(@(rho) 20 + rho(:,1))))
    verifyEqual(testCase, 3,    minNRho(pbcustom(@(rho) sin(rho(:,2)) + rho(:,1)*cos(rho(:,3)))));
    verifyEqual(testCase, 1000, minNRho(pbcustom(@(rho) rho(:,1) + 1*rho(:,1000)+1+rho(:,999)*rho(:,50))))
    
    %interp
    verifyEqual(testCase, 1,    minNRho(pbinterp({[1 2 3]}, [1 0 0])))
    verifyEqual(testCase, 2,    minNRho(pbinterp({[1 2 3], [4 5]}, [1 0 0; 1 2 3])))
end