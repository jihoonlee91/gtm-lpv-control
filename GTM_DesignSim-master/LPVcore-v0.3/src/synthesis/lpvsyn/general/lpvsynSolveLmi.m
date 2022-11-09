function [diagnostics,LMI,gamValue] = lpvsynSolveLmi(LMI,gam,var,synopts)
%LPVSYNSOLVELMI Solves the LMI for the controller synthesis problem.
%
%   Syntax:
%       [diagnostics,LMI,gamValue] = lpvsynSolveLmi(LMI,gam,var,synopts)
%
%   Inputs:
%       LMI (lmi): YALMIP LMI array containing LMI constraints for the
%           optimization problem.
%       gam (sdpvar): YALMIP sdpvar corresponding to the closed-loop gain 
%           objective variable, empty in case of passivity.
%       var (struct): Structure containing other optimization variables. 
%           (Only used for L1 based synthesis).
%       synopts (structure): Stucture containing LPV synthesis options, see
%           <a href="matlab:help lpvsynOptions">lpvsynOptions</a>.
%
%   Outputs:
%       diagnostics: Diagnostic output structure of YALMIP optimize 
%           function.
%       LMI: LMI array containing the LMI constraints for the controller
%           synthesis problem.
%       gamValue: Value of the closed-loop gain objective as double.
%   
%   See also LPVSYN.
%

%% Variables
perfType = synopts.performance;
solOpt = synopts.solverOptions;
l1Opt = synopts.l1Options;
verbose = synopts.verbose;
backoff = synopts.backoff;

%%
switch perfType
    %% L1 synthesis
    case 'l1'
        % Extract variables
        lambda = var.lambda;
        mu = var.mu;
        gamConstraint = sdpvar(1);
        LMI = LMI+(gam>=gamConstraint);
        l1Solve = optimizer(LMI,gam,solOpt,{lambda,gamConstraint},{gam,coefs(mu)});
        [gamValue,lambdaValue] = l1LineSearch(l1Solve,l1Opt,verbose,backoff);

        LMI = replace(LMI,lambda,lambdaValue);

        diagnostics.problem = 0;
        diagnostics.info = '';
    %% L2, passive and H2 synthesis
    otherwise
        diagnostics = optimize(LMI,gam,solOpt);
        gamValue = value(gam);
end
end