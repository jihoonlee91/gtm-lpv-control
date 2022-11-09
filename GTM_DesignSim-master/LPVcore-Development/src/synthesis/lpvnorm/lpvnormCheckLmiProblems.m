function [out,objValue] = lpvnormCheckLmiProblems(diagnostics,LMI,obj,objValue,opt)
%LPVNORMCHECKLMIPROBLEMS Checks if the LMI optimization problem returns
%errors corresponding to numerical problems, and if so, resolves the
%optimization problem using a fixed objective value. If there are no
%problems, the LMI is returned with backed off objective value assigned.
%
%   Syntax:
%       [out,objValue] = lpvnormCheckLmiProblems(diagnostics,LMI,obj,...
%                                                objValue,opt)
%
%   Inputs:
%       diagnostics (struct): Diagnostics information, assumed to be the
%           output of YALMIP optimize function. See <a href=
%           "matlab:help optimize">optimize</a>.
%       LMI (lmi): YALMIP LMI array containing LMI constraints for the
%           optimization problem.
%       obj (sdpvar): Objective of the optimization problem.
%       objValue (double): Value of the objective of the optimization problem.
%       opt (struct): Structure used with internal settings for <a href=
%           "matlab:help lpvnorm">lpvnorm</a>.
%
%   Outputs:
%       out: Boolean indicating if the LMI has correctly been solved.
%       objValue: (New) value of the objective.
%
%   See also LPVNORM.
%

%% Variables
solOpt = opt.SolverOptions;
type = opt.type;
backoff = 1.01;

%% Resolve LMI (if numerical issues)
if ~isempty(obj)
    if all(diagnostics.problem ~= [0 1])
        objValue = objValue*backoff;
        LMI = replace(LMI,obj,objValue);
        diagnostics = optimize(LMI,[],solOpt);
    end
end
out = lpvnormErrorCheck(diagnostics,type);
end
