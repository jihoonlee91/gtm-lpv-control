function [gamValue,lambdaValue,linfSolution,solverError] = linfLineSearch(linfSolve,linfOpt,verbose,backoff)
%LINFLINESEARCH Performe a line search for Linf based analysis and synthesis.
%
%   Syntax:
%       [gamValue,lambdaValue,linfSolution,solverError] = linfLineSearch(linfSolve,...
%                                                                     linfOpt,verbose)
%
%   Inputs:
%       linfSolve (optimizer): Optimizer object corresponding to the Linf
%           analysis or synthesis problem.
%       linfOpt (struct): Structure containing the options for Linf 
%           synthesis. See <a href="matlab:help linfOptions"
%           >linfOptions</a>.
%       verbose (logical): Logical indicating if progress messages should
%           be displayed (1) or not (0);
%
%   Outputs:
%       gamValue: Minimized induced Linf norm as double.
%       lambdaValue: Value of lambda as double corresponding to gamValue.
%       linfSolution: Output of optimizer corresponding to final iteration.
%       solverError: Logical indicating if problem has encountered an
%           error.
%
%   See also LPVSYN, LPVNORM.
%


%% Variables
lambdaGrid = linfOpt.lambdaGrid;
accLambda = linfOpt.accuracyLambda;
maxIt = linfOpt.maxIterations;

nLambda = 1; nGamma = 2;
nInit = numel(lambdaGrid);
solLambGamm = zeros(nGamma,nInit);
solLambGamm(nLambda,:) = lambdaGrid;

[linfSolution,solverProblem] = linfSolve(lambdaGrid,zeros(1,nInit));
solLambGamm(nGamma,:) = linfSolution{1};

solverError = false;    %   set to false (changed to true if error encountered)

% Resolve with backoff if there are numerical issues
if ~any(solverProblem == 0)
    resolveIndex = (solverProblem~=1) & (solverProblem~=0);
    if ~all(resolveIndex == 0)
        solLambGammResolve = solLambGamm(:,resolveIndex);
        [~,solverProblemResolve] = linfSolve(solLambGammResolve(nLambda,:),...
            solLambGammResolve(nGamma,:)*backoff);
        % Set solutions for gamma to resolved solutions
        solLambGamm(nGamma,resolveIndex) = solLambGammResolve(nGamma,:)*backoff;
        solverProblem(resolveIndex) = solverProblemResolve;
    end
end

% If no solution can be found error
if all(solverProblem~=0)
    solverError = true;
    warningMsg = ['No solution found during the initial line search for ',...
        'lambda. Solver returned error codes: ',num2str(solverProblem),...
        '. Consider increassing the number of points or changing the ',...
        'points of lambdagrid, see help linfOptions.'];
    warning(warningMsg);
    gamValue = [];
    lambdaValue = [];
    linfSolution = [];
    return;
end

% Find minimal value of gamma
[gamma0,idx] = min(solLambGamm(nGamma,:));
lambda0 = solLambGamm(nLambda,idx);

if verbose; fprintf('Initial search results:\n\t gamma=%.4g\t\tlambda=%.4g\n',gamma0,lambda0); end
if idx == 1
    lambdamax = solLambGamm(nLambda,idx+1);
    lambdamin = solLambGamm(nLambda,idx);
    solGammInit = solLambGamm(nGamma,[idx,idx+1]);
    warning(['Minimum gamma found at first index of initial lambda grid. '...
        'Consider lowering values of lambdagrid, see help linfOptions.'])
elseif idx == nInit
    lambdamax = solLambGamm(nLambda,idx);
    lambdamin = solLambGamm(nLambda,idx-1);
    solGammInit = solLambGamm(nGamma,[idx-1,idx]);
    warning(['Minimum gamma found at last index of initial lambda grid. '...
        'Consider increasing values of lambdagrid, see help linfOptions.'])
else
    lambdamax = solLambGamm(nLambda,idx+1);
    lambdamin = solLambGamm(nLambda,idx-1);
    solGammInit = solLambGamm(nGamma,[idx-1,idx+1]);
end
if verbose; fprintf('---------------------------------------------\n'); end
solGammInit(isnan(solGammInit)) = Inf;

% Golden ratio search
% See algorithm description here:
% https://gitlab.com/tothrola/LPVcore/-/issues/321
if verbose; fprintf('Golden section search:\n'); end
% if verbose; fprintf('Iter:\tGamma:\t\tLambda (min):\tLambda (max):\n\n'); end
if verbose; fprintf('Iter:\tLambda [Gamma] (min):\tLambda [Gamma] (max):\n\n'); end
gr = (sqrt(5) + 1) / 2; % Golden ratio
x1 = lambdamax-(lambdamax-lambdamin)/gr;
x2 = lambdamin + (lambdamax-lambdamin)/gr;

% Create lambda range using golden ratio
lambdaGrid = [lambdamin,x1, x2, lambdamax];
solGamm = zeros(1,4); solGamm([1,4]) = solGammInit;   % solGamm corresponds to gamma for lambda at lambdaGrid

% Determine gamma for the golden ratio range
[linfSolution,solverProblem] = linfSolve(lambdaGrid(2:3),[0 0]);
sol = linfSolution{1};
sol(solverProblem ~= 0) = Inf;  % Set to Inf if not solved correctly
solGamm(2:3) = sol; 
if verbose;fprintf('%i\t%.4g [%.4g]\t\t%.4g [%.4g]\n',0,lambdaGrid(1),solGamm(1),lambdaGrid(4),solGamm(4));end

cnt = 1;                % Reset counter
while ( ( ((lambdaGrid(4)-lambdaGrid(1)) > accLambda) && ...
        (abs(solGamm(1)-solGamm(4)) >= 1e-3) ) ||...
        all(isinf(solGamm([1,4]))) ) &&...
        (cnt <= maxIt)
    
    if all(isinf(solGamm(2:3)))
        lambdaGrid([1,4]) = (lambdaGrid([1,4])-lambda0)*.5+lambda0;
        lambdaGrid(2) = lambdaGrid(4)-(lambdaGrid(4)-lambdaGrid(1))/gr;
        lambdaGrid(3) = lambdaGrid(1) + (lambdaGrid(4)-lambdaGrid(1))/gr;
        [linfSolution,solverProblem] = linfSolve(lambdaGrid,[0 0 0 0]);
        solGamm = linfSolution{1};
        solGamm(solverProblem ~=0 ) = Inf; % Set to Inf if not solved correctly
    else
        % determine new lambda range
        [~,idx] = min(solGamm(2:3));
        % gamma @ lambdaGrid(2) < gamma @ lambdaGrid(3)
        if idx == 1
            % Update lambdaGrid and corresponding solGamm
            % Set new test point to NaN
            lambdaGrid = [lambdaGrid(1),NaN,lambdaGrid(2:3)];   
            solGamm = [solGamm(1),NaN,solGamm(2:3)];

            % Compute solution at new lambdaGrid(2)
            lambdaGrid(2) = lambdaGrid(4)-(lambdaGrid(4)-lambdaGrid(1))/gr;

            [linfSolution,solverProblem] = linfSolve(lambdaGrid(2),0);
            solGamm(2) = linfSolution{1};
            if (solverProblem ~= 0) || (solverProblem ~= 1)
                [linfSolution,solverProblem] = linfSolve(lambdaGrid(2),solGamm(2)*backoff);
            end
            if solverProblem ~= 0; solGamm(2) = Inf; end
        % gamma @ lambdaGrid(3) < gamma @ lambdaGrid(2)
        else
            % Update lambdaGrid and corresponding solGamm
            % Set new test point to NaN
            lambdaGrid = [lambdaGrid(2:3),NaN,lambdaGrid(4)];
            solGamm = [solGamm(2:3),NaN,solGamm(4)];

            % Compute solution at new lambdaGrid(3)
            lambdaGrid(3) = lambdaGrid(1) + (lambdaGrid(4)-lambdaGrid(1))/gr;

            [linfSolution,solverProblem] = linfSolve(lambdaGrid(3),0);
            solGamm(3) = linfSolution{1};
            if (solverProblem ~= 0) || (solverProblem ~= 1)
                [linfSolution,solverProblem] = linfSolve(lambdaGrid(3),solGamm(3)*backoff);
            end
            if solverProblem ~= 0; solGamm(3) = Inf; end
        end
    end
    if verbose;fprintf('%i\t%.4g [%.4g]\t\t%.4g [%.4g]\n',cnt,lambdaGrid(1),solGamm(1),lambdaGrid(4),solGamm(4));end
    cnt = cnt+1;
end

if all(isinf(solGamm))
    gamValue = gamma0;
    lambdaValue = lambda0;
else
    [gamValue,idx] = min(solGamm);
    lambdaValue = lambdaGrid(idx);
end
if verbose;fprintf('Golden section search results:\n\t gamma = %.4g\t\tlambda = %.4g\n',gamValue,lambdaValue);end
end