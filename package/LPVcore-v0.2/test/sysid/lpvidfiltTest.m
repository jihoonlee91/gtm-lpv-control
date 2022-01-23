function tests = lpvidfiltTest
%LPVIDPOLYTEST Test behavior of LPVIDPOLY
    tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(testCase) % Called once before the tests in this file are run
    p = preal('p', 'ct');
    testCase.TestData.p = p;
end

%% Test functions
function testCreate(testCase)
    filt = lpvidfilt({});
    verifyTrue(testCase, isempty(filt.Mat) && isempty(filt.Max) && ...
        isempty(filt.Min) && isempty(filt.Free));
end

function testDisp(~)
    p = preal('p', 'dt');
    % Non-monic
    disp(lpvidfilt({p, p^2}));
    % Monic
    disp(lpvidfilt({1, p^2}));
end

function testSlice(testCase)
    p = testCase.TestData.p;
    A = randn(2);
    filt = lpvidfilt({p * A, pdiff(p, 1) * randn(2)});
    filtSliced = slice(filt, 1, 1);
    mat = filt.Mat{1};
    matSliced = filtSliced.Mat{1};
    verifyTrue(testCase, mat(1, 1) == matSliced);
end

function testMinusOne(testCase)
    p = testCase.TestData.p;
    A = randn(2);
    filt = lpvidfilt({A, A * p + eye(2)});
    filtMinusOne = minusOne(filt);
    verifyTrue(testCase, filtMinusOne.Mat{1} == A - eye(2));
end

function testCat(testCase)
    p = preal('p', 'dt');
    A = lpvidfilt(1);
    B = lpvidfilt(p);
    C = [A, B];
    verifyTrue(testCase, isequal(C.Free{1}(:, :, 1), [1, 0]));
    verifyTrue(testCase, nparams(C, 'free') == 2);
    
    % Multi-dimensional example
    p = randn(2) * preal('p', 'dt');
    A = lpvidfilt({randn(2), 1 + p, p^2, p - p^2 + p^3});
    B = lpvidfilt({p, p - 1, p^7, p + p^3, randn(2), randn(2)});
    C = [A, B];
    % Note: the number of non-free parameters increases because the basis
    % functions are shared after concatenation. E.g. [1, p] --> 2 free
    % parameters, but 4 parameters in total because it can be written as
    %   [1 + 0 * p,  0 + 1 * p]
    verifyTrue(testCase, nparams(C, 'free') == ...
        nparams(A, 'free') + nparams(B, 'free'));
    verifyTrue(testCase, nparams(C) == ...
        nparams(A) + nparams(B) + 7 * 4);  % 7 hardcoded for this example, 
        % 4 because the dimension is 2-by-2
end

function testRand(testCase)
    A = lpvidfilt(randn(10));
    
    % Without interval should default to [0, 1]
    A = rand(A);
    verifyTrue(testCase, ...
        all(A.Value{:} <= 1, 'all') && all(A.Value{:} >= 0, 'all'));
    
    % With interval specified
    A = rand(A, [-0.25, 0.25]);
    verifyTrue(testCase, ...
        all(A.Value{:} <= 0.25, 'all') && all(A.Value{:} >= -0.25, 'all'));
    
    % Non-free parameters are unaffected
    A.Free{1} = false(size(A.Free{1}));
    Ar = rand(A, [-1000, 1000]);
    verifyEqual(testCase, A.Value, Ar.Value);
end

