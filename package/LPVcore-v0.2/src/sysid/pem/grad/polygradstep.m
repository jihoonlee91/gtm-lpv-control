function [cost, sys, delta, J, R] = polygradstep(sys, data, varargin)
%POLYGRADSTEP Return and apply gradient descent step for PEM objective in LPVIDPOLY estimation
%
%   Based on "Prediction error method for identification
%   of LPV models" by Yu Zhao et al. Calculates and applies a
%   gradient-based step for the PEM object in LPV-IO estimation. First, a search
%   direction is found (with support for regularization). Then, the
%   appropriate step size is selected by performing a bisection search,
%   halving the step size until a decrease in the cost function is
%   observed.
%
%   Syntax:
%       [cost, sys, delta, J, R] = polygrad(sys, data, Name, Value)
%
%   Inputs:
%       sys: lpvidpoly system
%       data: lpviddata object
%
%   Name-Value pairs:
%       'Lambda': regularization parameter for determination of R. The
%           larger lambda, the more the step will be in the direction opposite
%           the gradient.
%       'Mu': initial step size before bisection.
%       'MaxStepSizeBisectionAttempts': max. number of attempts to find a
%           step size for which the cost decreases. After each attempt, the
%           step size is halved. Default: 10.
%
%   Outputs:
%       cost: least squares prediction error cost
%       sys: updated lpvidpoly system
%       delta: update vector of the free parameters.
%       J: gradient of the free parameters in lpvidpoly with respect to
%       quadratic PEM objective.
%       R: matrix determining the search direction
%

y = data.y;
u = data.u;
p = data.p;
N = size(y, 1);

yhat = predict(sys, data, 1);

parser = inputParser();
addParameter(parser, 'Lambda', 1, @(x) isnumeric(x) && isscalar(x));
addParameter(parser, 'Mu', 1, @(x) isnumeric(x) && isscalar(x));
addParameter(parser, 'MaxStepSizeBisectionAttempts', 10, ...
    @(x) isnumeric(x) && isscalar(x) && x >= 0);
parse(parser, varargin{:});
lambda = parser.Results.Lambda;
mu = parser.Results.Mu;
maxStepSizeBisectionAttempts = parser.Results.MaxStepSizeBisectionAttempts;

%
%   Derivation of gradient: see "Prediction error method for identification
%   of LPV models", Yu Zhao et al. Note that their A is our F
%
%   Note that in (16), the partial derivative of the convergent joint of A
%   w.r.t. the parameter is wrong since non-commutativity is ignored.

%
%   The gradient of yhat w.r.t. parameter f from F is given by:
%
%       d/df yhat = -F+ * ( d/df F ) * F+ * B * u
%
%   with F+ the convergent joint of F (F+ * F == F * F+ == I).
%   This is solved by first simulating:
%
%       F s = B u
%
%   and then:
%
%       F (-d/df yhat) = (d/df F) s
%
%   for each parameter f.
%
A = sys.A;
F = sys.F;
B = sys.B;
C = sys.C;
D = sys.D;
H = lpvio(C.Mat, D.Mat);

% d/df yhat = -C+ * D * F+ * (d/df F) * F+ * B * u
% d/df yhat = -C+ * D * F+ * (d/df F) * s
% d/df yhat = -C+ * D * o
dF = parder(F);
syss = lpvio(F.Mat, B.Mat);
s = lsim(syss, p, u);
phiF = NaN(nparams(F, 'free'), N * sys.Ny);
for i=1:nparams(F, 'free')
    syso = lpvio(F.Mat, dF{i}.Mat);
    o = lsim(syso, p, s);
    % dyhat should be 1-by-(N * Ny) --> transposing and vectorization
    dyhat = -lsim(H, p, o)'; % Note transpose
    phiF(i, :) = dyhat(:)'; % Note transpose
end

% d/da yhat = -C+ * D * (d/da A) * y
% d/da yhat = -C+ * D * dAy
dA = parder(A);
phiA = NaN(nparams(A, 'free'), N * sys.Ny);
for i=1:nparams(A, 'free')
    sysdAy = lpvio(eye(sys.Ny), dA{i}.Mat);
    dAy = lsim(sysdAy, p, u);
    dyhat = -lsim(H, p, dAy)';
    phiA(i, :) = dyhat(:)';
end

% d/db yhat = -C+ * D * F+ * (d/db B) * u
% d/db yhat = -C+ * D * ds
dB = parder(B);
phiB = NaN(nparams(B, 'free'), N * sys.Ny);
for i=1:nparams(B, 'free')
    sysds = lpvio(F.Mat, dB{i}.Mat);
    ds = lsim(sysds, p, u);
    dyhat = lsim(H, p, ds)';
    phiB(i, :) = dyhat(:)';
end

% d/dc yhat  = - C+ * (d/c C) * C+ * D * ( F+ * B * u - A * y)
% d/dc yhat = - C+ * (d/dc C) * C+ * D * t
% d/dc yhat = -C+ * (d/dc C) * w
dC = parder(C);
phiC = NaN(nparams(C, 'free'), N * sys.Ny);
% Calculate auxiliary signals outside of loop (i-independent)
Ay = lsim(lpvio(eye(sys.Ny), A.Mat), p, y);
t = s - Ay;
w = lsim(H, p, t);
for i=1:nparams(C, 'free')
    dyhat = -lsim(lpvio(C.Mat, dC{i}.Mat), p, w)';
    phiC(i, :) = dyhat(:)';
end

% d/dd yhat = -C+ * (d/dd D) * (F+ * B * u - A * y)
% d/dd yhat = -C+ * (d/dd D) * t
dD = parder(D);
phiD = NaN(nparams(D, 'free'), N * sys.Ny);
for i=1:nparams(D, 'free')
    dyhat = -lsim(lpvio(C.Mat, dD{i}.Mat), p, w)';
    phiD(i, :) = dyhat(:)';
end

phi = [phiA; phiB; phiC; phiD; phiF]; % (nr. free parameters)-by-(N * Ny)

% e: N-by-Ny
e = y - yhat;
% N-by-Ny --> N * Ny-by-1
e = reshape(e', numel(e), 1);

J = - phi * e / N; % (nr. free parameters)-by-1
R = phi * phi' / N + lambda * eye(size(J, 1));

% Take successively smaller steps in search direction until lower cost is
% achieved
sys_candidate = sys;
c0 = e2cost(e);  % initial cost
alpha = 1;
for i=1:maxStepSizeBisectionAttempts+1
   delta = - alpha * mu * (R \ J);
    % Apply step to system
    sys_candidate = applydeltap(sys, delta);
    c = sys2cost(sys_candidate, data);
    % If the cost is decreased, the candidate system is selected
    if c < c0
        break;
    end
    alpha = 0.5 * alpha;
end

sys = sys_candidate;
cost = c;

end

function c = sys2cost(sys, data)
    % SYS2COST Takes system and dataset and returns Least Squares prediction error
    try
        ypred = predict(sys, data, 1);
    catch
        % Unstable system
        c = Inf;
        return;
    end
    ytrue = data.y;
    dy = ytrue - ypred;
    c = e2cost(dy);
end

function c = e2cost(dy)
    % E2COST Takes prediction error and returns Least Squares prediction
    % error
    c = 0.5 * sum(dy .* dy, 'all') / size(dy, 1);
end
    

