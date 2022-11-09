function p = preal(name, varargin)
%PREAL Assign a scalar parameter-varying variable
%
%   Syntax:
%       p = preal('p')
%       p = preal('p', 'Range', [-1, 1])
%       p = preal('p', 'ct', 'Unit', 'cm', 'Range', [-Inf, 100])
%       p = preal('p', 'ct', 'Range', [-5, 5], 'RateBound', [-1, 1])
%
%   Inputs:
%       name: name of the scalar parameter-varying variable
%       domain (char vec): time-domain. Either 'ct' or 'dt'. Default: 'ct'.
%       'Unit' (char vec): unit. Default: '-'.
%       'Range' (row vec): minimum and maximum value. Default: [-Inf, +Inf]
%       'RateBound' (row vec): minimum and maximum rate change. Default:
%           [-Inf, +Inf]
%
%   Outputs:
%       p (pmatrix): scalar pmatrix object.
%
%   See also PMATRIX
    
    in = inputParser;
    in.addOptional('domain', 'ct', @(x)any(strcmpi(x, {'ct', 'dt'})));
    in.addParameter('Dynamic', 0, @isnumeric);
    in.addParameter('Unit', '-', @ischar);
    in.addParameter('Range', [-Inf, +Inf], @isnumeric);
    in.addParameter('RateBound', [-Inf, +Inf], @isnumeric);
    in.parse(varargin{:});
    
    domain = lower(in.Results.domain);
    dynamic = in.Results.Dynamic;
    unit = in.Results.Unit;
    range = in.Results.Range;
    rateBound = in.Results.RateBound;
    
    p = pmatrix(1, ...
        'BasisType', 'affine', ...
        'BasisParametrization', {1}, ...
        'SchedulingDomain', domain, ...
        'SchedulingMap', dynamic, ...
        'SchedulingName', {name}, ...
        'SchedulingUnit', {unit}, ...
        'SchedulingRange', {range}, ...
        'SchedulingRateBound', {rateBound});

end

