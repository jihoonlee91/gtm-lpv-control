function varargout = subsref(obj, S)
%SUBSREF Subscripted referencing for lpvlfr objects.
%
%   F = P(i:j, k:l)
%
%   Inputs:
%       P: indexed lpvlfr object.
%       S: indexing structure.
%
%   Outputs:
%       F: result of indexing expression.
%
%   This method overloads the builtin SUBSREF functionality for the case of
%   2-dimensional parentheses indexing. In this case, it indexes the
%   input and output channels of the lpvlfr object. The third
%
%   See also SUBSASGN, LPVLFR.

% Only subsref of the following form are overloaded: P(x, y)
% This implies that S.type == '()' with 2 subscripts S.subs.
if numel(S) == 1 && strcmp(S.type, '()') && numel(S.subs) == 2
    ysubs = S.subs{1};
    usubs = S.subs{2};
    S.subs{1} = horzcat(1:obj.Nz, obj.Nz + S.subs{1});
    S.subs{2} = horzcat(1:obj.Nw, obj.Nw + S.subs{2});
    G = subsref(obj.G, S);
    obj.G_ = G;
    obj.InputName = obj.InputName(usubs);
    obj.InputUnit = obj.InputUnit(usubs);
    obj.InputDelay = obj.InputDelay(usubs);
    obj.OutputName = obj.OutputName(ysubs);
    obj.OutputUnit = obj.OutputUnit(ysubs);
    varargout{1} = obj;
else
    [varargout{1:nargout}] = builtin('subsref', obj, S);
end