function tf = iscellof(A, type)
%ISCELLOF Determine if input is cell array where each element is of
%specified type
%
%   Syntax:
%       tf = iscellof(A, type)
%       tf = iscellof(A, fn)
%
%   Inputs:
%       A: cell array
%       type: string denoting the type each element of A should have
%       fn: validator function handle that takes as input an element of A
%           and outputs logical 1 if the element has the correct type and
%           logical 0 otherwise.
%
%   Outputs:
%       tf: true if A is cell array of specified type, false otherwise.
%

if ~iscell(A)
    tf = false;
    return;
end

if isa(type, 'function_handle')
    % type is a validator function handle
    fn = type;
    tf = all(cellfun(@(x) fn(x), A));
else
    % type is a string denoting the class
    tf = all(cellfun(@(x) isa(x, type), A));
end

end

