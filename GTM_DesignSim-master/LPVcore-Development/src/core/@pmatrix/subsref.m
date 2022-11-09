function varargout = subsref(P, S)
%SUBSREF Subscripted referencing for pmatrix objects.
%
%   F = P(i:j, k:l)
%
%   Inputs:
%       P: indexed pmatrix object.
%       S: indexing structure.
%
%   Outputs:
%       F: result of indexing expression.
%
%   This method overloads the builtin SUBSREF functionality for the case of
%   2-dimensional parentheses indexing. In this case, it indexes the
%   matrices of the pmatrix object in the first 2 dimensions. The third
%   dimensions is untrimmed, so that all basis functions are preserved.
%
%   See also SUBSASGN, PMATRIX.

% Only subsref of the following form are overloaded:
%
%   indexing: P(x, y)
%   vectorization: P(:)
%

if numel(S) == 1 && strcmp(S.type, '()') && numel(S.subs) <= 2 ...
        && ~(numel(S.subs) == 1 && isa(S.subs{1}, 'char'))  % ':'
    % Indexing
    r = P;
    [n, m, o] = size(P.matrices);
    % Linear indexing
    if numel(S.subs) == 1
        [row, col] = ind2sub([n, m], S.subs{1});
        S.subs = {row, col};
    end
    S.subs{3} = 1:o;
    r.matrices = subsref(P.matrices, S);
    varargout{1} = r;
elseif numel(S) == 1 && strcmp(S.type, '()') ...
        && numel(S.subs) == 1 && S.subs{1} == ':'
    % Vectorization
    r = P;
    o = size(P.matrices, 3);
    r.matrices = reshape(P.matrices, [], 1, o);
    varargout{1} = r;
else
    [varargout{1:nargout}] = builtin('subsref', P, S);
end