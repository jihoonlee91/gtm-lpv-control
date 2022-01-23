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

function testMinNRho(testCase)
    %doc verifyEqual specifies actual first then expected. Here however
    %they are reversed to increase readabillity.
    
    %Constants should have 0 required inputs to evaluate
    verifyEqual(testCase, 0, minNRho(pbconst()))
    verifyEqual(testCase, 0, minNRho(pbcustom(@(rho) 1)))
    
    %poly:
    verifyEqual(testCase, 3, minNRho(pbaffine(3)))
    verifyEqual(testCase, 3, minNRho(pbpoly([1 0 0 0; 0 0 2 0])))
    a = zeros(100);
    a([1:5 16 18 30], [1:2 5 20 50]) = 1;
    verifyEqual(testCase, 50, minNRho(pbpoly(a)))
    
    %custom:
    verifyEqual(testCase, 1,    minNRho(pbcustom(@(rho) 20 + rho(:,1))))
    verifyEqual(testCase, 3,    minNRho(pbcustom(@(rho) sin(rho(:,2)) + rho(:,1)*cos(rho(:,3)))));
    verifyEqual(testCase, 1000, minNRho(pbcustom(@(rho) rho(:,1) + 1*rho(:,1000)+1+rho(:,999)*rho(:,50))))

end