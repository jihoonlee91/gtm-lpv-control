function p = pmatCellCommonBfuncs(p)
%pmatCellCommonBfuncs Equalize basis functions across cell array of pmatrix
%objects
%
%   Syntax:
%       pc = pmatCellCommonBfuncs(p)
%
%   Inputs:
%       p: cell array of pmatrix objects with possibly different basis
%           functions.
%
%   Outputs:
%       pc: cell array of same size as input, with each pmatrix object in
%           the cell array having the same basis functions. Added basis
%           functions are assigned the coefficient 0.
%
%   Example:
%       p = preal('p', 'dt')
%       q = preal('q', 'dt')
%       pmatCellCommonBfuncs({p, q})
%

%% Input validation
assert(iscell(p), 'input must be a cell array');
assert(iscellof(p, 'pmatrix'), 'input must be a cell array of pmatrix objects');

%% Algorithm
if numel(p) == 0; return; end

% pRef is the reference pmatrix object containing all the basis functions
pRef = p{1};
for i=2:numel(p)
    pRef = simplify(commonbfuncs_(p{i}, pRef));
end

% Apply basis functions in pRef to all pmatrix objects
for i=1:numel(p)
    p{i} = simplify(commonbfuncs_(simplify(p{i}), pRef));
end

end

