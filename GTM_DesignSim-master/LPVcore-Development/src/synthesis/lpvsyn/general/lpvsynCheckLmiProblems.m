function [LMI, flag] = lpvsynCheckLmiProblems(diagnostics,LMI,obj,objValue,synopts,invertBackoff)
%LPVSYNCHECKLMIPROBLEMS Checks if the LMI optimization problem returns
%errors corresponding to numerical problems, and if so, resolves the
%optimization problem using a fixed objective value. If there are no
%problems, the LMI is returned with backed off objective value assigned.
%
%   Syntax:
%       LMI = lpvsynCheckLmiProblems(diagnostics,LMI,obj,synopts,...
%                                    flipBackoff)
%
%   Inputs:
%       diagnostics (struct): Diagnostics information, assumed to be the
%           output of YALMIP optimize function. See <a href=
%           "matlab:help optimize">optimize</a>.
%       LMI (lmi): YALMIP LMI array containing LMI constraints for the
%           optimization problem.
%       obj (sdpvar): Objective of the optimization problem.
%       synopts (struct): LPV synthesis option structure, see also
%           <a href="matlab:help lpvsynOptions">lpvsynOptions</a>.
%       invertBackoff (logical): Logical indicating if the used backoff 
%           value should be inverted (1) or not (0), i.e., if invertBackoff
%           is true, backoff := 1/backoff.
%
%   Outputs:
%       LMI: LMI array containing the new LMI constraints for the
%           optimization problem.
%       flag: True if LMI is resolved, false otherwise.
%
%   See also LPVSYN.
%

backoff = synopts.backoff;
solOpt = synopts.solverOptions;

if invertBackoff
    backoff = 1/backoff;
end

flag = false;

if ~isempty(obj)
    objValue = objValue*backoff;
    % Only resolve with backoff if the previous optimization problem is not
    % feasible or infeasible (hence, there were probably numerical issues)
    if all(diagnostics.problem ~= [0 1])
        LMI = replace(LMI,obj,objValue);
        diagnostics = optimize(LMI,[],solOpt);
        flag = true;
    % Otherwise just fix the objective (with backoff)
    else
        LMI = replace(LMI,obj,objValue);
    end
end
assert(diagnostics.problem == 0,lpvsynErrorMessage(diagnostics));
end
