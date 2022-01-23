function [At,st] = pmatrixToRolmip(A,varargin)
%PMATRIXTOROLMIP converts a pmatrix object with affine parameter dependency
%to a rolmip object.
%
%   Syntax:
%       [At, st] = pmatrixToRolmip(A)
%       
%       [At, st] = pmatrixToRolmip(A,name)
%
%   Inputs: 
%       A (pmatrix): Affine parameter-varying matrix that is converted to
%           rolmip object.
%       name (char): Optional argument that for the label of the rolmip
%           object.
%       
%   Outputs:
%       At: Rolmip object of the matrix A.
%       st: Cell array indicating the parameter dependency of the rolmip
%           object. Usefull when constructing a rolmip variable with the
%           same dependency. See polmask in <a href="matlab:help rolmipvar">rolmipvar</a>.
%

    %% Pars input
    switch numel(varargin)
        case 0
            matrixName = inputname(1);
        case 1
            matrixName = varargin{1};
        otherwise
            error('Too many input arguments');
    end
    
    %% Checks on matrix
    % TODO (09/04/2021): Add support for polynomial parameter dependency
    assert(isa(A,'pmatrix'),'Matrix is not a pmatrix object.');
    assert(isaffine(A),'Matrix not affine.');
    
    %% Create simplified version to check if it is constant
    As = simplify(A,eps);
    
    %% Constant case
    if isconst(As)
        At = rolmipvar(As.matrices,matrixName,0,0);
        st = {};
    %% Parameter-varying case
    else
        % Run simplify such that constant part is first basis functions and
        % the degrees of the bfuncs are correct. Add zero such that it 
        % always contains a constant part.
        A = simplify(A+0);   

        np = A.Np;

        % "st" variable used for rolmip to indicate where the sched.var
        % dependency is located 
        st = cell(np+1,1);  st{1} = zeros(1,np);
        A_ = cell(1,np+1);  A_{1} = {st{1}, A.matrices(:,:,1)};
        for ii = 2:np+1 % [A0 + p1 A1 + ... +pnp Anp]
            % build correct setup for rolmip
            st{ii} = A.bfuncs{ii}.degrees;
            % Pad with 0s to ensure each st{ii} has same size
            st{ii} = [st{ii}, zeros(1, np-numel(st{ii}))];
            A_{ii} = {st{ii},A.matrices(:,:,ii)};
        end
        rhoRange = reshape(cell2mat(A.timemap.Range'),2,[])';
        At = rolmipvar(A_,matrixName,rhoRange);
    end
end