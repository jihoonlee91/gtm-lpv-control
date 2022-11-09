function v = validatepmatrixCellArray(C)
%VALIDATEPMATRIXCELLARRAY Validates whether each entry in cell array is a
%pmatrix with compatible dimensions.
%
%   This function is used to validate LPVIO and LPVIDOLY objects.
%
%   Syntax:
%       v = LPVcore.validatepmatrixCellArray(C)
%
%   Inputs:
%       C: cell array of PMATRIX objects.
%
%   Outputs:
%       v: logical indicating whether all objects in C have the same first
%       two dimensions.
%

sz1 = cellfun(@(x) size(x, 1), C);
sz2 = cellfun(@(x) size(x, 2), C);

v = false;
if ~isempty(sz1) && ~all(sz1 == sz1(1))
    return;
end
    
if ~isempty(sz2) && ~all(sz2 == sz2(1))
    return;
end

v = true;

end

