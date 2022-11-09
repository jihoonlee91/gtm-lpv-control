function p = randpmatrix(n, m, o, varargin)
%RANDPMATRIX Generates random pmatrix object.
%
%   Syntax:
%       p = randpmatrix(n, m, o, ...)
%
%   Inputs:
%       n, m (int): size of matrices.
%       o (int): number of matrices.
%
%   Outputs:
%       p (pmatrix): randomly generated pmatrix object.
%
%   Alternative usage:
%       b = randpmatrix(a)
%
%   Inputs:
%       a: pmatrix
%
%   Outputs:
%       b: pmatrix with random coefficient and basic functions and size of
%       a.
%

if isa(n, 'pmatrix')
    mat = n.matrices;
    bfuncs = n.bfuncs;
    tm = n.timemap;
    p = pmatrix(randn(size(mat)), bfuncs, 'SchedulingTimeMap', tm);
else
    p = pmatrix(randn(n, m, o), varargin{:});
end

end

