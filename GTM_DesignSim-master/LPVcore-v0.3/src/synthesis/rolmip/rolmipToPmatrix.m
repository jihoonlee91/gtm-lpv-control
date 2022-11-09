function Ap = rolmipToPmatrix(A,B,st)
%ROLMIPTOPMATRIX Converts rolmip (with degree 1) variable to affine pmatrix
%object.
%
%   Syntax:
%       Ap = rolmipToPmatrix(P,B,st)
%
%   Inputs: 
%       A (rolmipvar): Rolmip variable that needs to be converted to a
%           pmatrix object.
%       B (pmatrix): Pmatrix variable from which to extract the scheduling-
%           variables to construct Ap.
%       st (cell): Cell array indicating the parameter dependency of the 
%           rolmip object. See also polmask in <a href="matlab:help
%           rolmipvar">rolmipvar</a> and 
%           <a href="matlab:help pmatrixToRolmip">pmatrixToRolmip</a>. 
%               
%   Outputs:
%       Ap: Pmatrix object corresponding to the rolmip object
%
%   See also PMATRIX, ROLMIPVAR.


%% Checks on matrix
assert(isa(A,'rolmipvar') | isa(A,'double'),'Matrix is not a rolmipvar object.');
assert(isa(B,'pmatrix'),'Second argument is not a pmatrix object.');
assert(isaffine(B),'Second argument is not affine pmatrix object.');

%% Compute Ap
A = value(A);   % Convert to rolmip entries to doubles

if isa(A,'double')
    % Constant case
    Ap = A;
else
    % Parameter-varying case
    B = simplify(0+B);
    np = B.Np;

    npA = size(st,1)-1;

    Av = cell(np,1);
    Av{1} = evalpar(A,st{1});
    Ap = Av{1};
    for i = 1:npA
        Av{i+1} = evalpar(A,st{i+1})-Av{1};
        p = pmatrix(1,B.bfuncs(find(st{i+1})+1),'SchedulingTimeMap',B.timemap);
        Ap = Ap+value(Av{i+1})*p;
    end
    Ap = simplify(Ap,eps);   % Remove unnecessary scheduling dependency
end
