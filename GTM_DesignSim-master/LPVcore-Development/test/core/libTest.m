function tests = libTest
%PMATRIXTEST Test behavior of various functions in "src/lib"
tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(~) % Called once before the tests in this file are run
end

%% Test functions
function testIsInt(testCase)
    shouldBeTrue = {-1, 0, 1, 1E5};
    for i=1:numel(shouldBeTrue)
        verifyTrue(testCase, isint(shouldBeTrue{i}));
        % Non-double input
        verifyTrue(testCase, isint(int8(shouldBeTrue{i})));
        verifyTrue(testCase, isint(uint8(shouldBeTrue{i})));
        verifyTrue(testCase, isint(int64(shouldBeTrue{i})));
        verifyTrue(testCase, isint(uint64(shouldBeTrue{i})));
    end
    
    shouldBeFalse = {'', -Inf, Inf, NaN, eye(2), randn(3, 6), ...
        pmatrix(1), 1.2, @(x) sin(x)};
    for i=1:numel(shouldBeFalse)
        verifyFalse(testCase, isint(shouldBeFalse{i}));
    end
end

function testPmatCellCommonBfuncs(testCase)
    verifyError(testCase, @() pmatCellCommonBfuncs('h'), '');
    verifyError(testCase, @() pmatCellCommonBfuncs({'h'}), '');
    p = preal('p', 'dt');
    q = preal('q', 'dt');
    % out should be {p + 0 * q, 0 * p + q}
    out = pmatCellCommonBfuncs({p, q});
    verifyEqual(testCase, numel(out), 2);
    %% TODO: fix this after #295 is fixed
    % verifyEqual(testCase, out{1}.numbfuncs, 2);
    verifyEqual(testCase, out{2}.numbfuncs, 2);
end

function testCellArrayProperty(testCase)
    verifyError(testCase, @() LPVcore.cellarrayProperty('t', 1, 1), '');
    verifyError(testCase, @() LPVcore.cellarrayProperty('t', {1, 2}, 1), '');
    verifyError(testCase, @() LPVcore.cellarrayProperty('t', {1, 2}, 2), '');
    verifyError(testCase, @() LPVcore.cellarrayProperty('t', ...
        {'a', 'b'; 'c', 'd'}, 4), '');
    verifyError(testCase, @() LPVcore.cellarrayProperty('t', ...
        {'a', 'b'}, 3), '');
    % String is not supported
    verifyError(testCase, @() LPVcore.cellarrayProperty('t', ...
        {"a", "b"}, 2), ''); %#ok<CLARRSTR>
    % Empty
    verifyEqual(testCase, LPVcore.cellarrayProperty('t', {}, 0), {});
    
    verifyEqual(testCase, LPVcore.cellarrayProperty('t', ...
        {'a'}, 3), {'a(1)'; 'a(2)'; 'a(3)'});
    verifyEqual(testCase, LPVcore.cellarrayProperty('t', ...
        'a', 3), {'a(1)'; 'a(2)'; 'a(3)'});
    verifyEqual(testCase, LPVcore.cellarrayProperty('t', ...
        'a', 1), {'a'});
    verifyEqual(testCase, LPVcore.cellarrayProperty('t', ...
        {'a', 'b', 'c'}, 3), {'a'; 'b'; 'c'});
    verifyEqual(testCase, LPVcore.cellarrayProperty('t', ...
        {'a'; 'b'; 'c'}, 3), {'a'; 'b'; 'c'});
end

function testIsCellOf(testCase)
    % Non-cell inputs
    verifyFalse(testCase, iscellof(1, 'double'));
    verifyFalse(testCase, iscellof([1, 2, 3], 'double'));
    verifyFalse(testCase, iscellof("he", 'double'));
    verifyFalse(testCase, iscellof('he', 'double'));
    
    % Empty cell
    verifyTrue(testCase, iscellof({}, 'double'));
    verifyTrue(testCase, iscellof({}, 'char'));
    verifyTrue(testCase, iscellof({}, 'string'));

    % Simple types
    verifyTrue(testCase, iscellof({1}, 'double'));
    verifyTrue(testCase, iscellof({1, 2, 3}, 'double'));
    verifyTrue(testCase, iscellof({'h', 'e'}, 'char'));
    verifyFalse(testCase, iscellof({1}, 'char'));
    verifyFalse(testCase, iscellof({'h', 'e'}, 'double'));
    verifyFalse(testCase, iscellof({{}}, 'string'));
    
    % Function handle validation
    verifyTrue(testCase, iscellof({1, 2, 3}, @(x) isint(x)));
    verifyFalse(testCase, iscellof({1, 2.2, 3}, @(x) isint(x)));
    
    % With nested cell arrays
    verifyTrue(testCase, ...
        iscellof({{1}, {2}, {3}}, @(x) iscellof(x, 'double')));
end

function testPevalMulti(testCase)
    p = preal('p', 'dt');
    A = 2 * p;
    
    N = 100;
    Ts = 1;
    p = randn(N, 1);
    
    % Simple case: static dependency
    [psimulti, pmulti, tmulti] = pevalmulti(A, p);
    [~, psi] = peval(A, p);
    
    verifyEqual(testCase, pmulti, p);
    verifyEqual(testCase, tmulti, (1:N)');
    verifyEqual(testCase, psimulti, psi);
    
    % Multiple input pmatrix objects
    [psi1, psi2, psi3, pmulti] = pevalmulti(A, A, A, p);
    verifyEqual(testCase, psi1, psi2);
    verifyEqual(testCase, psi2, psi3);
    verifyEqual(testCase, pmulti, p);
    
    % Only pmatrix objects with same scheduling signal are accepted
    p = preal('p', 'dt');
    q = preal('q', 'dt');
    verifyError(testCase, @() pevalmulti(p, q, 1), '');
    
    % Time vector must be valid
    verifyError(testCase, @() pevalmulti(p, [1; 2], [2; 1]), '');
    verifyError(testCase, @() pevalmulti(p, [1; 2], [1; 2; 3]), '');
    verifyError(testCase, @() pevalmulti(p, [1; 2; 3], [1; 2; 4]), '');
    
    % Num. columns of p must equal dim. of scheduling signal
    p = preal('p', 'dt');
    verifyError(testCase, @() pevalmulti(p, [1, 1]), '');
    
    % Dynamic dependence
    p0 = preal('p', 'dt');
    p2 = pshift(p0, 2);
    pData = randn(N, 1);
    t = (1:Ts:N)';
    [psi1, psi2, pDataOut, tOut] = pevalmulti(p0, p2, pData, t);
    
    % Compatible sizes
    verifySize(testCase, psi1, [N-2, 1]);
    verifySize(testCase, psi2, [N-2, 1]);
    verifySize(testCase, pDataOut, [N-2, 1]);
    % Check truncated values
    verifyEqual(testCase, pDataOut, pData(1:end-2));
    verifyEqual(testCase, psi1, pDataOut);
    verifyEqual(testCase, psi2, pData(3:end));
    verifyEqual(testCase, tOut, t(1:end-2));
    
    % Continuous-time
    p = preal('p', 'ct');
    pd = pdiff(p, 1);
    pdd = pdiff(p, 2);
    
    [psi1, psi2, psi3, pOut, tOut] = pevalmulti(p, pd, pdd, pData, t);
    
    [~, psi1Ref] = peval(p, pData, Ts);
    [~, psi2Ref] = peval(pd, pData, Ts);
    [~, psi3Ref] = peval(pdd, pData, Ts);
    
    verifyEqual(testCase, pOut, pData);
    verifyEqual(testCase, tOut, t);
    verifyEqual(testCase, psi1, psi1Ref);
    verifyEqual(testCase, psi2, psi2Ref);
    verifyEqual(testCase, psi3, psi3Ref);
end

function testAreBFuncsEqual(testCase)
    p = preal('p', 'dt');
    q = preal('q', 'dt');
    
    verifyTrue(testCase, areBFuncsEqual(p, p));
    verifyTrue(testCase, areBFuncsEqual(1+p, 1+p+p));
    
    verifyFalse(testCase, areBFuncsEqual(p, p^2));
    verifyFalse(testCase, areBFuncsEqual(p, q));
end

function testSignalPropertiesFuncs(testCase)
    % Test functions related to handling signal properties (io, state)
    G1 = rss(10, 1, 1);
    G2 = rss(10, 1, 1);
    % Check whether LPVcore.doSignalPropertiesMatch works correctly
    % Two modes: 'io' (default) only checks input-output signal properties,
    % 'full' also checks state dimension.
    verifyTrue(testCase, LPVcore.doSignalPropertiesMatch(G1, G2));
    verifyTrue(testCase, LPVcore.doSignalPropertiesMatch(G1, G2, 'io'));
    verifyTrue(testCase, LPVcore.doSignalPropertiesMatch(G1, G2, 'full'));
    G2.InputName = 'new-name';
    verifyFalse(testCase, LPVcore.doSignalPropertiesMatch(G1, G2));
    % Check whether copying the signal properties leads to same signal
    % properties again
    G2 = LPVcore.copySignalProperties(G2, G1);
    verifyTrue(testCase, LPVcore.doSignalPropertiesMatch(G1, G2, 'full'));
    % Ensure error is thrown when incompatible systems are compared
    verifyError(testCase, @() LPVcore.doSignalPropertiesMatch(...
        rss(10, 1, 1), rss(10, 2, 2)), '');
    verifyError(testCase, @() LPVcore.doSignalPropertiesMatch(...
        rss(10, 1, 1), rss(20, 1, 1), 'full'), '');
    verifyError(testCase, @() LPVcore.copySignalProperties(...
        rss(10, 1, 1), rss(10, 2, 2)), '');
    verifyError(testCase, @() LPVcore.copySignalProperties(...
        rss(10, 1, 1), rss(20, 1, 1), 'full'), '');
end