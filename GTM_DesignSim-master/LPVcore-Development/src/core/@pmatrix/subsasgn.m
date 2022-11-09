function varargout = subsasgn(A, S, B)
%SUBSASGN Subscripted assignment for pmatrix
%
%   Syntax:
%       A = subsasgn(A, S, B)
% 
%   See also: SUBSREF, PMATRIX
%

% subsasgn is implemented as an addition. Consider this assignment:
%   B(x, y) = C
% This can be written as a summation:
%   B = CLifted + Mask .* B
% Where Mask(x, y) = 0, 1 everywhere else.
% Followed by simplify to group identical basis functions:
%   B = simplify(B, 'poly')

if numel(S) == 1 && strcmp(S.type, '()') && numel(S.subs) <= 2
    % Size of target pmatrix segment
    [sz1, sz2] = size(A);
    % Convert linear index to row-column index
    if numel(S.subs) == 1
        if S.subs{1} == ':'
            row = 1:sz1;
            col = 1:sz2;
        else
            [row, col] = ind2sub([sz1, sz2], S.subs{1});
        end
        S.subs = {row, col};
    elseif S.subs{1} == ':'
        S.subs{1} = 1:sz1;
    elseif S.subs{2} == ':'
        S.subs{2} = 1:sz2;
    end
    if isa(B, 'pmatrix')
        % Lift the matrices of B into the size of the target pmatrix
        % segment
        BLifted = B;
        mat = B.matrices;
        o = size(B, 3);
        matLifted = NaN(max([sz1;S.subs{1}(:)]), max([sz2;S.subs{2}(:)]), o);
        for i=1:o
            matLifted(:, :, i) = builtin('subsasgn', zeros(sz1, sz2), S, mat(:, :, i));
        end
        BLifted.matrices = matLifted;
    else
        % B is already a matrix, simply lift into target size
        BLifted = builtin('subsasgn', zeros(sz1, sz2), S, B);
    end
    padding1 = max(S.subs{1})-sz1;
    padding2 = max(S.subs{2})-sz2;
    
    A = [A, zeros(sz1, padding2); zeros(padding1, sz2), zeros(padding1, padding2)];
    Mask = builtin('subsasgn', ones(size(A)), S, 0);
    varargout{1} = Mask .* A + BLifted;
else
    [varargout{1:nargout}] = builtin('subsasgn', A, S, B);
end

end


