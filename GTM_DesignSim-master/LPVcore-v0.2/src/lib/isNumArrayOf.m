function tf = isNumArrayOf(A, type)
%ISCELLOF Determine if input is numeric array where each element is of
%specified type
%
%   Syntax:
%       tf = isNumArrayOf(A, type)
%       tf = isNumArrayOf(A, fn)
%
%   Inputs:
%       A: numeric array
%       type: string denoting the type each element of A should have
%       fn: validator function handle that takes as input an element of A
%           and outputs logical 1 if the element has the correct type and
%           logical 0 otherwise.
%
%   Outputs:
%       tf: true if A is matrix with elements of specified type, false otherwise.
%

if ~isnumeric(A)
    tf = false;
    return;
end

if isa(type, 'function_handle')
    % type is a validator function handle
    fn = type;
    tf = all(arrayfun(@(x) fn(x), A));
else
    % type is a string denoting the class
    tf = all(arrayfun(@(x) isa(x, type), A));
end

end

