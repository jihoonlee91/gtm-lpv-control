function sysest = lpvpolypseudols_(data, sysest, options, gui)
%LPVPOLYPSEUDOLS_ Estimate a general LPVIDPOLY model from data using
%pseudo-linear least squares
%
%   sys = lpvpolypseudols_(data, init_sys)
%      estimates an LPVIDPOLY model from data using the input-ouput data
%      DATA.
%
%   INIT_SYS is an LPVIDPOLY object. This object contains the free to be
%      estimated parameters.
%   DATA is the time-domain estimation data captured in an LPVIDDATA
%      object. Type "help lpviddata" for more information.
%
%   sys = lpvpolypseudols_(data, init_sys, options)
%      specifies the options of the estimation methodology. Use the
%      "lpv{armax/oe/bj/polyest}Options" command to configure OPTIONS.
%
% See also LPVPOLYEST, LPVPOLYESTOPTIONS

%
% Iterative LS global method (see 'Prediction Error
% Identification of LPV Systems: Present and Beyond', below Algorithm 3).
%
%   LPVIDPOLY model:
%
%       A y = (F \ B) u + (D \ C) e
%
%   Define noise-free output ytilde = (F \ B) u
%
%   Predictor has following form:
%
%       yhat = (C \ D) (F \ B) u + (1 - (C \ D) A) y
%
%   Left-multiply predictor (2.61) with C:
%
%       C yhat = D ytilde + (C - D A ) y       (2.62)
%
%   Equivalent to (eps := y - yhat):
%
%       eps = (1 - C) eps + D (A y - ytilde)  (2.63)
%
%   D is monic. Write as D = 1 + (D - 1) and split:
%
%       eps = A y - ytilde + (1 - C) eps + (D - 1) (A y - ytilde)
%
%   Substitute ytilde = (1 - F) ytilde + B u, and let sigma := A y - ytilde
%
%       eps = A y - (1 - F) ytilde - B u + (1 - C) eps + (D - 1) sigma
%
%   Using eps = y - yhat, we can write yhat as a linear function of A,
%   ..., F:
%
%       yhat = (1-A)y + Bu + (C-1)eps + (D-1)(-sigma) + (F-1)(-ytilde)
%
%   y and u are known from the data. This corresponds to the following
%   augmented ARX system:
%
%       yhat = (1-A)y + [B, C-1, D-1, F-1] [u; eps; -sigma; -ytilde]
%
%   ytilde, eps, sigma are calculated if A, B, C, D, F are fixed.
%   A, B, C, D, F are estimated if ytilde, eps, sigma are fixed.
%   This leads to an iterative LS global method.
%
%   A, B, C, D, F are estimated using arx_ for the following system:
%
%       A y = [B, C - 1, D - 1, F - 1] [u; eps; -sigma; -ytilde]
%
%   ytilde, eps, sigma are generated using one-step ahead prediction:
%
%       ytilde = predict(lpvidpoly(F, B))
%       sigma = y - ytilde
%       yp = predict(lpvidpoly(A, B, C, D, F))
%       eps = y - yp
%
%   Note: F - 1 = minusOne(lpvidpoly.F)
%

optionsRegul = lpvarxRegulOptions;

ny = sysest.Ny;
nu = sysest.Nu;

Na = sysest.Na;
Nb = sysest.Nb;
Nc = sysest.Nc;
Nd = sysest.Nd;
Nf = sysest.Nf;

y = data.OutputData;
p = data.SchedulingData;
u = data.InputData;

%% Initialization
maxIter = options.SearchOptions.MaxIterations;
fTol = options.SearchOptions.FunctionTolerance;
sTol = options.SearchOptions.StepTolerance;
s = Inf;

% Initial estimate of an ARX system
CMinusOne = zeros(minusOne(sysest.C));
DMinusOne = zeros(minusOne(sysest.D));
if isarmax(sysest)
    initArxSysest = arx_(...
        lpvidpoly(sysest.A, sysest.B, {}, {}, {}, ...
        sysest.NoiseVariance, sysest.Ts, 'InputDelay', sysest.InputDelay), ...
        data, options, optionsRegul);
    A = initArxSysest.A;
    FMinusOne = minusOne(sysest.F);
elseif isbj(sysest)
    initArxSysest = arx_(lpvidpoly(sysest.F, sysest.B, {}, {}, {}, ...
        sysest.NoiseVariance, sysest.Ts, 'InputDelay', sysest.InputDelay), ...
        data, options, optionsRegul);
    A = sysest.A;
    FMinusOne = minusOne(initArxSysest.A);
end
Btilde = [initArxSysest.B, CMinusOne, DMinusOne, FMinusOne];

arxsysest = lpvidpoly(A, Btilde, [], [], [], sysest.NoiseVariance, ...
    sysest.Ts, 'InputDelay', [sysest.InputDelay; zeros(3 * ny, 1)]);
% Get initial cost
f = Inf;

% For arx_, the Display should be turned off
arxOptions = options;
arxOptions.Display = 'off';

for iter=1:maxIter
    %% Update LPVIDPOLY system
    A = arxsysest.A;
    Btilde = arxsysest.B;
    CMinusOne = slice(Btilde, [], nu+1:nu+ny);
    DMinusOne = slice(Btilde, [], nu+ny+1:nu+2*ny);
    FMinusOne = slice(Btilde, [], nu+2*ny+1:nu+3*ny);
    sysest.A = truncate(A, Na + 1);
    sysest.B = truncate(slice(Btilde, [], 1:nu), Nb + 1);
    sysest.C = truncate(plusOne(CMinusOne), Nc + 1);
    sysest.D = truncate(plusOne(DMinusOne), Nd + 1);
    sysest.F = truncate(plusOne(FMinusOne), Nf + 1);

    %% Estimation of eps, sigma, ytilde
    [yp, ytilde] = predict(sysest, data, 1);
    eps = y - yp;   % eq. (2.18)
    sigmaSys = lpvidpoly(pidpoly(eye(ny), [], [], [], true), ...
        [sysest.A, pidpoly(eye(ny))]);
    sigma = lsim(sigmaSys, p, [y, -ytilde]);
    fprev = f;
    f = cost(sysest, data);
    sprev = s;
    s = getpvec(sysest);
    step = norm(sprev - s);
    gui = update(gui, iter, f, step);
    % Function tolerance termination
    if abs(fprev - f) < fTol
        terminate(gui, sysest, SysIdTermConditions.FunctionTolerance);
        return;
    end
    % Step tolerance termination
    if step^2 / numel(s) < sTol
        terminate(gui, sysest, SysIdTermConditions.StepTolerance);
        return;
    end
    
    %% ARX identification of A, B, C, D, F
    arxdata = lpviddata(y, p, [u, eps, -sigma, -ytilde]);
    arxsysest = arx_(arxsysest, arxdata, arxOptions, optionsRegul);
end

terminate(gui, sysest, SysIdTermConditions.MaxIterationsReached);

function c = cost(sysest_, data_)
    % COST Returns current cost of estimated system w.r.t. data
    [yp_, ~] = predict(sysest_, data_, 1);
    eps_ = data_.y - yp_;   % eq. (2.18)
    N_ = size(eps_, 1);
    c = norm(eps_)^2 / N_;
end

end

