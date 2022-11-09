function out = lpvsynOptions(varargin)
%LPVSYNOPTIONS Create LPV synthesis options structure.
%
%   Syntax:
%       out = lpvsynOptions(...)
%   
%   Optional name-value pairs:
%       'Performance': Perfomance metric that is considered for synthesis.
%           Possible values: 'l1', 'l2', 'h2', 'passive'. 
%           Default value: 'l2'.
%       'Backoff': Correction factor for gamma, used in the synthesis to 
%           reduce numerical errors, since the true optimal solution is
%           close to singular. 
%           Default value: 1.01.
%       'SolverOptions': Structure containing the YALMIP solver settings. 
%           See <a href="matlab:help sdpsettings">sdpsettings</a>. 
%           Default value: sdpsettings('verbose',0,'solver','sdpt3').
%       'NumericalAccuracy': Numerical accuracy parameter. Used for solving
%           the LMIs (LMI <= -NumAcc). 
%           Default value: 1e-8.
%       'Method': Indicates what method to use for synthesis. Possible
%           values: 'transformation', 'projection'. 
%           Default value: 'transformation'.
%       'ParameterVaryingStorage': Indicates if a parameter-varying (1) or
%           parameter independent (0) storage function is considered. If
%           set to [], lpvsyn will check if an parameter-varying storage
%           function can be used or not.
%           Default value: [].
%       'L1Options': Structure containing the options for L1 synthesis. See
%           <a href="matlab:help l1Options">l1Options</a>.
%           Default value: l1Options().
%       'Dependency': Array (1x4 or 4x1) indicating which controller
%           matrices, corresponding to [A,B,C,D], should have parameter-
%           dependency. In case of the transformation method, these apply
%           to the transformed controller matrices.
%           Default value: [1 1 1 1].
%       'Verbose': Turns progress messages on (1) or off (0);
%           Default value: 1;
%       'DirectFeedthrough': Option indicating if controller has direct
%           feedthrough (1) or not (0).
%           Default value: 1;
%       'ImproveConditioning': Solves extra optimization problems to
%           improve the conditioning of found solutions.
%           Default value: 1;
%       'PoleConstraint': Structure containing the matrices describing the
%           constraints on the closed-loop eigenvalues, see
%           <a href="matlab:help poleConstraintOptions">poleConstraintOptions</a>. If set to [], no constraints are
%           considered.
%           Default value: [].
%       
%   Outputs:
%       out: Structure containing LPV synthesis options.
%   
%   See also LPVSYN.
%

%% Input parser
% Default values
performance = 'l2';
backoff = 1.01;
solverOptions = sdpsettings('verbose',0,'solver','sdpt3');
numAcc = 1e-8;
method = 'transformation';
parVarSt = [];     % 0 = constant X, 1 = parameter varying X, [] = unset (lpvsyn does check)
l1Opt = l1Options;
dependency = [1 1 1 1];
verbose = 1;
directFeedthrough = 1;
improveCond = 1;
poleConstraint = [];

% Parse
opt = inputParser;

addParameter(opt,'Performance',performance,@performanceCheck);
addParameter(opt,'Backoff',backoff,@positiveCheck);
addParameter(opt,'SolverOptions',solverOptions,@isstruct);
addParameter(opt,'NumericalAccuracy',numAcc,@positiveCheck);
addParameter(opt,'Method',method,@methodCheck);
addParameter(opt,'ParameterVaryingStorage',parVarSt,@isnumeric);
addParameter(opt,'L1Options',l1Opt,@isstruct)
addParameter(opt,'Dependency',dependency,@dependencyCheck);
addParameter(opt,'Verbose',verbose,@isnumeric);
addParameter(opt,'DirectFeedthrough',directFeedthrough,@isnumeric);
addParameter(opt,'ImproveConditioning',improveCond,@isnumeric);
addParameter(opt,'PoleConstraint',poleConstraint,@checkPoleConstraint);

parse(opt,varargin{:});

synopts.performance             = opt.Results.Performance;
synopts.backoff                 = opt.Results.Backoff;
synopts.solverOptions           = opt.Results.SolverOptions;
synopts.numericalAccuracy       = opt.Results.NumericalAccuracy;
synopts.method                  = opt.Results.Method;
synopts.parameterVaryingStorage = logical(opt.Results.ParameterVaryingStorage);
synopts.l1Options               = opt.Results.L1Options;
synopts.dependency              = logical(opt.Results.Dependency);
synopts.directFeedthrough       = logical(opt.Results.DirectFeedthrough);
synopts.verbose                 = logical(opt.Results.Verbose);
synopts.improveConditioning     = logical(opt.Results.ImproveConditioning);
synopts.poleConstraint          = opt.Results.PoleConstraint;

synopts.solverOptions.cachesolvers = 1;  % enable caching of solvers for small speed-up

%% Output
out = synopts;
end

%% Checking functions
function out = performanceCheck(performance)
    possiblePerformance = {'l1','l2','h2','passive'};
    out = any(strcmp(performance,possiblePerformance));
end

function out = methodCheck(method)
    possibleMethod = {'projection','transformation'};
    out = any(strcmp(method,possibleMethod));
end

function out = dependencyCheck(dependency)
    out = all(size(reshape(dependency,1,[])) == [1,4]);
end

function out = positiveCheck(value)
    if isempty(value)
        out = true;
    else
        out = isnumeric(value) && (value > 0);
    end
end

function out = checkPoleConstraint(value)
    out = isempty(value) || all(matches(fieldnames(value),{'Q','S','T','U'}));
end