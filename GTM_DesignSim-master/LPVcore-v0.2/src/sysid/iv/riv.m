function [sys, info, ysim] = riv(data, sys, tol, max_iter) 
%RIV Identification of LPV-BJ model using Refined IV method
%
%	See for further explanations:
%	Laurain et al,
%	"Refined Instrumental variables for identification of LPV models",
%	Automatica
%
%   Syntax:
%       [sys, info, ysim] = riv(data, init_sys, tol, max_iter)
%
%   Inputs:
%       data: lpviddata object containing IO data
%       sys: lpvidpoly object from which the basis functions and structure
%           is used, but not the values of the coefficients.
%       tol: tolerance on relative variations of the estimated parameters.
%           Used to stop the identification (default: 1E-4).
%       max_iter: max. number of iterations (default: 20).
%
%   Outputs:
%       sys: estimated lpvidpoly object.
%       info: information on the identification procedure.
%       ysim: simulated output to the inputs specified in data for the
%       estimated system.
%
%   Original algorithm (for SISO) contributed to LPVcore by:
%
%       Vincent Laurain
%       CRAN - Centre de Recherche en Automatique de Nancy
%       e-mail : vincent.laurain@cran.uhp-nancy.fr
%

%% Input validation
narginchk(2, 4);
assert(isa(data, 'lpviddata'), '''data'' must be an LPVIDDATA object');
assert(isa(sys, 'lpvidpoly') && isbj(sys) && issiso(sys)  && isdt(sys), ...
    '''sys'' must be a discrete-time SISO LPV-BJ model structure');
if nargin <= 2; tol = 1E-4; end
if nargin <= 3; max_iter = 20; end

% Constant noise process
assert(isconst(sys.C), '''C'' of the model structure must be LTI');
assert(isconst(sys.D), '''D'' of the model structure must be LTI');

% Identical parametrization of B and F
if sys.F.N > 1
    assert(areBFuncsEqual(sys.B.Mat{:}, sys.F.Mat{2:end}), ...
        'Coefficients of ''B'' and ''F'' of the model structure must have identical scheduling signals and basis functions');
else
    assert(areBFuncsEqual(sys.B.Mat{:}), ...
        'Coefficients of ''B'' of the model structure must have identical scheduling signals and basis functions');
end

%% Algorithm
[data,nn,pA,pB,Ae,Be] = lpvcore2riv(data, sys);
[~, Ae, Be, Ce,De,info,ysim]=rivLPV(data,nn,tol,max_iter,pA,pB,Ae,Be);
sys = riv2lpvcore(sys, Ae, Be, Ce, De);

end

