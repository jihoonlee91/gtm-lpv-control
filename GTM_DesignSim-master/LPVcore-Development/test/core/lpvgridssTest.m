function tests = lpvgridssTest
%LPVGRIDSSTEST Test behavior of LPVGRIDSS
    tests = functiontests(localfunctions);
end

%% File fixtures
function setupOnce(~) % Called once before the tests in this file are run
end

%% Test functions
function testCreation(testCase)
    InputName = {'a', 'b'};
    OutputName = {'y1', 'y2', 'y3'};
    nu = numel(InputName);
    ny = numel(OutputName);
    % Each local system must have same number of states
    sys(:, :, 1) = rss(10, ny, nu);
    sys(:, :, 2) = rss(6, ny, nu);
    grid = struct('p', [1, 2]);
    verifyError(testCase, @() lpvgridss(sys, grid), '');
    % grid and sys must be compatible
    sys(:, :, 2) = rss(10, ny, nu);
    verifyError(testCase, @() lpvgridss(sys, ...
        struct('p', [1, 2, 3])), '');
    verifyError(testCase, @() lpvgridss(sys, ...
        struct('p', [1, 2], 'q', [1, 2])), '');
    verifyError(testCase, @() lpvgridss(sys, ...
        struct('q', 1)), '');
    lpvsys = lpvgridss(sys, grid);
    % Interpolation method must be supported
    verifyError(testCase, @() lpvinterpss(lpvsys, 'linear'), '');
    % Sample time and IO names matches
    globalsys = lpvinterpss(lpvsys, 'pwlinear');
    verifyEqual(testCase, globalsys.Ts, sys.Ts);
    verifyEqual(testCase, globalsys.InputName, sys.InputName);
    verifyEqual(testCase, globalsys.OutputName, sys.OutputName);
    % System plots
    bode(lpvsys);
    sigma(lpvsys);
    bodemag(lpvsys);
    close;
    disp(lpvsys);
end

function testInterconnection(testCase)
    % Test interconnection functions
    sys(:, :, 2) = rss(10, 1, 1);
    sys(:, :, 1) = rss(10, 1, 1);
    % Define 2 lpvgridss objects with incompatible grid
    s1 = lpvgridss(sys, struct('p', [0, 1]));
    s2 = lpvgridss(sys, struct('q', [0, 1]));
    % Verify that the interconnection with s1 and s2 always fail (due to
    % incompatible grid)
    verifyError(testCase, @() [s1, s2], '');
    verifyError(testCase, @() [s1; s2], '');
    verifyError(testCase, @() blkdiag(s1, s2), '');
    verifyError(testCase, @() append(s1, s2), '');
    verifyError(testCase, @() lft(s1, s2), '');
    verifyError(testCase, @() feedback(s1, s2), '');
    verifyError(testCase, @() series(s1, s2), '');
    verifyError(testCase, @() parallel(s1, s2), '');

    % Define 2 lpvgridss objects with compatible grid
    s2 = lpvgridss(sys, struct('p', [0, 1]));
    % Check that interconnection works same as built-in commands
    funcs = {@horzcat, @vertcat, @blkdiag, @append, @lft, @feedback, @series, @parallel};
    for i=1:numel(funcs)
        func = funcs{i};
        s = func(s1, s2);
        verifyEqual(testCase, s.ModelArray, func(sys, sys));
    end
end

function testInterpolation(testCase)
    % Test whether interpolation is done correctly by comparing with an
    % LPV-SS representation that can be represented exactly as an LPV
    % grid-based SS model.
    rng(1);
    %% Configuration
    nx = 4;
    ny = 1;
    nu = 1;
    
    %% Build LPV-SS model
    p = preal('p', 'ct');
    q = preal('q', 'ct');
    A = -nx * eye(nx) + p * rand(nx) + q * rand(nx);
    B = randn(nx, nu) + p * randn(nx, nu) + q * randn(nx, nu);
    C = randn(ny, nx) + p * randn(ny, nx) + q * randn(ny, nx);
    D = randn(ny, nu) + p * randn(ny, nu) + q * randn(ny, nu);
    lpvsys = lpvss(A, B, C, D, 0);
    
    %% Sample from LPV-SS model
    pGrid = linspace(-1, 0, 3);
    qGrid = linspace(-1, 0, 3);
    sys(:, :, numel(pGrid), numel(qGrid)) = ...
        extractLocal(lpvsys, [pGrid(end), qGrid(end)]);
    for i=1:numel(pGrid)
        for j=1:numel(qGrid)
            sys(:, :, i, j) = extractLocal(lpvsys, [pGrid(i), qGrid(j)]);
        end
    end
    grid = struct('p', pGrid, 'q', qGrid);
    gridsys = lpvgridss(sys, grid);

    lpvgridsys1 = lpvinterpss(gridsys, 'pwlinear');
    lpvgridsys2 = lpvinterpss(gridsys, 'behavioral', ...
        struct('init_sys', lpvsys, ...
        'hinfstructOptions', hinfstructOptions('MaxIter', 100, 'Display', 'off')));
    lpvgridsys3 = lpvinterpss(gridsys, 'poly', ...
        struct('p', 2, 'q', 2));
    lpvgridsys4 = lpvinterpss(gridsys, 'pwpoly', 'spline');
    verifyEqual(testCase, lpvgridsys1.Ts, lpvsys.Ts);
    verifyEqual(testCase, lpvgridsys2.Ts, lpvsys.Ts);
    verifyEqual(testCase, lpvgridsys3.Ts, lpvsys.Ts);
    verifyEqual(testCase, lpvgridsys4.Ts, lpvsys.Ts);
    
    %% Validate results
    pGridVal = linspace(-1, 0, 7);
    qGridVal = linspace(-1, 0, 7);
    for i=1:numel(pGridVal)
        for j=1:numel(qGridVal)
            sysTrue = extractLocal(lpvsys, [pGridVal(i), qGridVal(j)]);
            sys1 = extractLocal(lpvgridsys1, [pGridVal(i), qGridVal(j)]);
            sys2 = extractLocal(lpvgridsys2, [pGridVal(i), qGridVal(j)]);
            sys3 = extractLocal(lpvgridsys3, [pGridVal(i), qGridVal(j)]);
            verifyEqual(testCase, hinfnorm(sysTrue - sys1), 0, ...
                'AbsTol', 1E-10);
            % Tolerance is lower because behavioral approximation takes a
            % long time (so the number of iterations was limited above).
            verifyEqual(testCase, hinfnorm(sysTrue - sys2), 0, ...
                'AbsTol', 0.02);
            verifyEqual(testCase, hinfnorm(sysTrue - sys3), 0, ...
                'AbsTol', 1E-10);
        end
    end
end

function testConnect(testCase)
    %% Example from GitLab issue #363
    % See https://gitlab.com/tothrola/LPVcore/-/issues/363

    rng(1);
    G = rss(2,1,1,3);
    grid = struct('p',1:3);
    G.InputName = 'u';
    G.OutputName = 'y';
    
    Gg = lpvgridss(G,grid);
    
    Pg = connect(Gg, sumblk('z = y + d'), {'d','u'}, {'z'});
    P = connect(G, sumblk('z = y + d'), {'d','u'}, {'z'});
    
    verifyTrue(testCase, all(hinfnorm(P-Pg.ModelArray) < 1E-10));
    verifyTrue(testCase, LPVcore.doSignalPropertiesMatch(P, Pg.ModelArray))

    %% Test case when first and third argument are lpvgridss
    G2 = G;
    G2.InputName = 'z';
    G2.OutputName = 'o';

    Gg2 = Gg;
    Gg2.InputName = 'z';
    Gg2.OutputName = 'o';

    Pg = connect(Gg, sumblk('z = y + d'), Gg2, {'d','u'}, {'o'});
    P = connect(G, sumblk('z = y + d'), G2, {'d','u'}, {'o'});


    verifyTrue(testCase, all(hinfnorm(P-Pg.ModelArray) < 1E-10));
    verifyTrue(testCase, LPVcore.doSignalPropertiesMatch(P, Pg.ModelArray))
end
