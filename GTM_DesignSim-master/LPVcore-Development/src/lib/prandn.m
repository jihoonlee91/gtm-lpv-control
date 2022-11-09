function P = prandn(varargin)
%PRANDN Create constant pmatrix with random coefficients

mat = randn(varargin{:});
P = pmatrix(mat, ...
    'SchedulingDomain', 'undefined', ...
    'SchedulingMap', [], ...
    'SchedulingDimension', 0);
end

