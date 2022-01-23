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
%           close to singular. Default value: 1.01.
%       'SolverOptions': Structure containing the YALMIP solver settings. 
%           See also <a href="matlab:help sdpsettings">sdpsettings</a>. Default value: 
%           sdpsettings('verbose',0,'solver','mosek').
%       'NumericalAccuracy': Numerical accuracy parameter. Used for solving
%           the LMIs (LMI<= NumAcc). Default value: 1e-8.
%       'L2Method': Parameter indicating which synthesis method is used.
%           The parameter 1 or true indicates synthesis based on
%           transformation, 0 or false indicates synthesis based on
%           projection. Default value: 1.
%       'ParameterVaryingStorage': Indicates whether a parameter-varying (1)
%           or parameter independent (0) storage function is considered.
%           Default value: 0.
%       'L1Options': Structure containing the options for L1 synthesis. See
%           also <a href="matlab:help l1Options">l1Options</a>.
%   
%   Outputs:
%       out: Structure containing LPV synthesis options.
%

%% Input parser
% Default values
performance = 'l2';
backoff = 1.01;
solverOptions = sdpsettings('verbose',0);
numAcc = 1e-8;
l2method = 1;       % 0 = projection, 1 = transformation
parVarSt = 0;       % 0 = constant X, 1 = parameter varying X
l1Opt = l1Options; 

% Parse
opt = inputParser;

addParameter(opt,'Performance',performance,@performanceCheck);
addParameter(opt,'Backoff',backoff,@isnumeric);
addParameter(opt,'SolverOptions',solverOptions,@isstruct);
addParameter(opt,'NumericalAccuracy',numAcc,@isnumeric);
addParameter(opt,'L2Method',l2method,@isnumeric);
addParameter(opt,'ParameterVaryingStorage',parVarSt,@isnumeric);
addParameter(opt,'L1Options',l1Opt,@checkStruct)

parse(opt,varargin{:});

synopts.performance             = opt.Results.Performance;
synopts.backoff                 = opt.Results.Backoff;
synopts.solverOptions           = opt.Results.SolverOptions;
synopts.numericalAccuracy       = opt.Results.NumericalAccuracy;
synopts.l2Method                = logical(opt.Results.L2Method);
synopts.parameterVaryingStorage = opt.Results.ParameterVaryingStorage;
synopts.l1Options               = opt.Results.L1Options;

out = synopts;
end

function out = performanceCheck(performance)
    possiblePerformance = {'l1','l2','h2','passive'};
    out = nnz(strcmp(performance,possiblePerformance)) == 1;
end