function tests = idpmatrixTest
%IDPMATRIXTEST Test behavior of IDPMATRIX
tests = functiontests(localfunctions);
end

function testIdPmatrix(testCase)
    p = preal('p', 'dt');
    pi = idpmatrix(p);
    verifyEqual(testCase, nparams(pi), 1);
    verifyEqual(testCase, nparams(pi, 'free'), 1);
    verifyEqual(testCase, pi.matrices, 1);

    % Setting coefficients to non-free
    pi = idpmatrix(p, false);
    verifyEqual(testCase, nparams(pi), 1);
    verifyEqual(testCase, nparams(pi, 'free'), 0);

    % Matrix-valued case
    pi = idpmatrix(p * eye(2) + randn(2), false);
    verifyEqual(testCase, nparams(pi), 8);
    verifyEqual(testCase, nparams(pi, 'free'), 0);

    free = cat(3, false(2), true(2));
    pi = idpmatrix(p * eye(2) + randn(2), free);
    verifyEqual(testCase, nparams(pi), 8);
    verifyEqual(testCase, nparams(pi, 'free'), 4);

    % Direct syntax
    pi = idpmatrix(cat(3, randn(3), randn(3)), [], [], [], {pbconst, pbaffine(1)});
    verifyEqual(testCase, nparams(pi), 18);
    verifyEqual(testCase, nparams(pi, 'free'), 18);
end
