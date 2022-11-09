function mat = getMatrices(A,varargin)
% GETMATRICES Get matrices of gridvar object, as nD matrix or
% cell. Returns nD matrix by default.
%
%   Syntax:
%       mat = getMatrices(A)
%       mat = getMatrices(A,'matrix')
%       mat = getMatrices(A,'cell')
%
%   Inputs:
%       Ag (gridvar): Gridvar object.
%       
%   Outputs:
%       mat: Matrices of the gridvar object.
%
if isa(A,'gridvar')
    if nargin == 1
        mat = reshape([A.matrices{:}],[size(A),A.dim]);
    elseif nargin == 2
        switch varargin{1}
            case 'matrix'
                mat = reshape([A.matrices{:}],[size(A),A.dim]);
            case 'cell'
                mat = A.matrices;
            otherwise
                error(["Unrecognized option, available options",...
                    " are 'cell' and 'matrix', see help",...
                    " getMatrices."]);
        end
    else
        error('Too many input arguments');
    end
else
    mat = A;
end
end