function varargout = subsasgn(A, S, B)
%SUBSASGN Subscripted assignment for gridvar
%
%   Syntax:
%       A = subsasgn(A, S, B)
% 
%   See also: SUBSREF, GRIDVAR
%

% subsasgn is implemented as an addition. Consider this assignment:
%   B(x, y) = C
% This can be written as a summation:
%   B = CLifted + Mask .* B
% Where Mask(x, y) = 0, 1 everywhere else.
% Followed by simplify to group identical basis functions:
%   B = simplify(B, 'poly')

if numel(S) == 1 && strcmp(S.type, '()') && numel(S.subs) <= 2
    r = A;
    r.matrices = cellfun(@(A) subsasgn(A,S,B),A.matrices,'UniformOutput',false); %#ok<SUBSASGN> 
    varargout{1} = r;
else
    [varargout{1:nargout}] = builtin('subsasgn', A, S, B);
end

end


