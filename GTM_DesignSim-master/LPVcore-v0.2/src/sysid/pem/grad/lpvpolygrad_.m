function sys = lpvpolygrad_(data, init_sys, options, gui)
%LPVPOLYGRAD Gradient-based estimation of LPVIDPOLY system
%
%   Syntax:
%       sys = lpvpolygrad(data, init_sys, options, gui)
%
%   Inputs:
%       data: lpviddata object
%       init_sys: lpvidpoly object that indicates the structure of the
%           model to be identified.
%       options: lpv{armax/oe/bj/polyest}Options object.
%       gui: graphical user interface (when display option is 'on')
%
%   Outputs:
%       sys: estimated system.
%

Verbose = options.Verbose;
Lambda = options.Regularization.Lambda;
Mu = options.SearchOptions.StepSize;
MaxIter = options.SearchOptions.MaxIterations;
FuncTol = options.SearchOptions.FunctionTolerance;
StepTol = options.SearchOptions.StepTolerance;

sys = init_sys;
cprev = cost(sys, data);
h = gui;
if Verbose; fprintf('Initial cost: %.2f\n', cprev); end
for i=1:MaxIter
    [c, sys, delta] = polygradstep(sys, data, 'Lambda', Lambda, 'Mu', Mu);
    step = norm(delta);
    h = update(h, i, c, step);
    if Verbose; fprintf('Cost after iteration %d / %d: %.2d\n', i, MaxIter, c); end
    if cprev - c <= FuncTol
        terminate(h, sys, SysIdTermConditions.FunctionTolerance);
        return;
    end
    cprev = c;
    if step <= StepTol
        terminate(h, sys, SysIdTermConditions.StepTolerance);
        return;
    end
end
terminate(h, sys, SysIdTermConditions.MaxIterationsReached);

end

function c = cost(sys, data)
    % COST Least Squares cost on Prediction Error
    ypred = predict(sys, data, 1);
    ytrue = data.y;
    N = size(ytrue, 1);
    dy = ytrue - ypred;
    c = 0.5 * sum(dy .* dy, 'all') / N;
end

