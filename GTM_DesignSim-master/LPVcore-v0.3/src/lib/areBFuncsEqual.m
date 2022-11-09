function tf = areBFuncsEqual(varargin)
%AREBFUNCSEQUAL Return whether pmatrix objects in input have same basis
%functions
%
%   Syntax:
%       tf = areBFuncsEqual(p1, p2, ..., pn)
%
%   Inputs:
%       p1, ..., pn: pmatrix objects
%
%   Outputs:
%       tf: logical indicating whether the input pmatrix objects have the
%           same ext. scheduling signal and basis functions.
%

%% Input validation
narginchk(1, Inf);
assert(iscellof(varargin, 'pmatrix'), 'inputs must be pmatrix objects');
p = varargin;

%% Algorithm
% Set tf to false so we can return at any point
if numel(p) == 1
    tf = true;
    return;
else
    tf = false;
end

% Check equality in timemap and number of basis functions
N = numel(p{1}.bfuncs);
tm = p{1}.timemap;
for i=2:numel(p)
    if tm ~= p{i}.timemap || N ~= numel(p{i}.bfuncs)
        return;
    end
end

% Check basis function equality
for i=1:N
    bf = p{1}.bfuncs{i};
    for j=2:numel(p)
        if bf ~= p{j}.bfuncs{i}
            return
        end
    end
end

% Set tf to true since all checks passed
tf = true;

end

