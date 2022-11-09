function out = l1Options(varargin)
%L1OPTIONS Create L1 options structure.
%
%   Syntax:
%       out = l1Options(...)
%   
%   Optional name-value pairs:
%       'LambdaGrid': Array of values for lambda used for initial search.
%           Default value: linspace(1e-8, 5, 15).
%       'AccuracyLambda': Maximum difference between lambda search interval
%           during optimization. Default value: 1e-5.
%       'MaxIterations': Maximum number of iterations for optimizing lambda.
%   
%   Outputs:
%       out: Structure containing L1 options.
%

%% Input parser
l1Opts.lambdaGrid = linspace(1e-8, 5, 15);  % Initial lambda search grid 
l1Opts.accuracyLambda = 1e-5;               % Maximum difference between lambda search interval
l1Opts.maxIterations = 50;                          % Maximum number of iterations

opt = inputParser;

greaterThanZero = @(x) x>0;
lambdaGridCheck = @(x) (min(size(x)) == 1) && all(x>0);

addParameter(opt,'LambdaGrid',l1Opts.lambdaGrid,lambdaGridCheck);
addParameter(opt,'AccuracyLambda',l1Opts.accuracyLambda,greaterThanZero);
addParameter(opt,'MaxIterations',l1Opts.maxIterations,greaterThanZero);

parse(opt,varargin{:});

l1Opts.lambdaGrid = opt.Results.LambdaGrid;
l1Opts.accuracyLambda = opt.Results.AccuracyLambda;
l1Opts.maxIterations = opt.Results.MaxIterations;

out = l1Opts;
end