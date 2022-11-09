function sys = lpvssest_egn_(sys, data, options)
%EGN Enhanced Gauss-Newton based search of DDLC parameterized LPV-SS identification [1]
%
%   [1] Towards efficient identification of linear parameter-varying
%   state-space models (Cox, P. B. 2018)
%
%   Name-Value pairs:
%       'LambdaMin'
%       'Beta1': step size. Default: 1
%       'MaxIterations': maximum number of gradient iterations. Default:
%       100
%       'FunctionTolerance': termination tolerance on the loss function
%       that is minimized, specified as a positive scalar. Default: 1E-5.
%       'StepTolerance': termination tolerance on the estimated parameter
%       values, specified as a positive scalar. Default: 1E-6.
%       'Display': whether to plot identification progress in a window.
%           Accepted values: 'on' or 'off'.
%

if nargin <= 2
    options = lpvssestOptions;
end
beta1 = options.SearchOptions.beta;
gamma = options.SearchOptions.gamma;
lambda_min = options.SearchOptions.lambdaMIN;
alpha_min = options.SearchOptions.alphaMIN;
v = options.SearchOptions.nu;
epsilon = options.SearchOptions.epsilon;
maxIter = options.SearchOptions.maxIter;

lambda = 0;
k = 0;

assert(strcmpi(typeNoiseModel(sys), 'innovation'), ...
    'Only innovation form LPVIDSS models are supported.');
nx = sys.Nx;
nu = sys.Nu;
ny = sys.Ny;
ts = sys.Ts;
u = data.u;
y = data.y;
p = data.p;
N = size(p, 1);

doTerminate = false;

h = progressgui(sys, data, options);

while ~doTerminate
    k = k + 1;
    yp = predict(sys, data, 1);
    e = y - yp;  % (7.5)
    %
    %   Parameter vector theta is split into components:
    %
    %       theta = [thetaA; thetaB; thetaC; thetaD; thetaK; thetaX0]
    %
    %   Each element of theta corresponds to one column in the Jacbian matrix J.
    %   J can also be split into components related to the matrices and init.
    %   state:
    %
    %       J = [JA, JB, JC, JD, JK, JX0]
    %
    %   Each row in J represents a timestep with the value of de / dtheta:
    %
    %       de / dtheta = - [ dyhat_1 / dtheta; ...; dyhat_N / dtheta]
    %
    %   dyhat can be calculated using the following LPV-SS model:
    %
    %       dxhat / dtheta[k+1] = [ A - K * C ] * dxhat / dtheta ...
    %           + [ d(A - K * C)/dtheta, d(B - K * D)/dtheta, dK/dtheta] *
    %               [xhat; u; y]
    %
    %       yhat[k+1] = C dxhat / dtheta + [ dC / dtheta, dD/dtheta, 0 ] * 
    %               [xhat; u; y]
    %
    %   which simplifies for each component in theta.
    %

    % A and C matrices do not depend on partial derivatives w.r.t. theta
    A = (sys.A - sys.K * sys.C);
    C = sys.C;

    %% First, we calculate xhat (7.3a)
    Axhat = A;
    Bxhat = [sys.B - sys.K * sys.D, sys.K];
    sysxhat = lpvss(Axhat, Bxhat, eye(nx), zeros(nx, nu + ny), ts);
    xhat = lsim(sysxhat, p, [u, y]);

    %% thetaA
    der = parder(sys.Structure.A);
    n = nparams(sys.Structure.A, 'free');
    JA = NaN(N, n);
    for i=1:n
        B = der{i};
        D = [];
        syss = lpvss(A, B, C, D, ts);
        yhat = lsim(syss, p, xhat)';
        JA(:, i) = -yhat(:)';
    end

    %% thetaB
    der = parder(sys.Structure.B);
    n = nparams(sys.Structure.B, 'free');
    JB = NaN(N, n);
    for i=1:n
        B = der{i};
        D = [];
        syss = lpvss(A, B, C, D, ts);
        yhat = lsim(syss, p, u)';
        JB(:, i) = -yhat(:)';
    end

    %% thetaC
    der = parder(sys.Structure.C);
    n = nparams(sys.Structure.C, 'free');
    JC = NaN(N, n);
    for i=1:n
        B = -sys.K * der{i};
        D = der{i};
        syss = lpvss(A, B, C, D, ts);
        yhat = lsim(syss, p, xhat)';
        JC(:, i) = -yhat(:)';
    end

    %% thetaD
    der = parder(sys.Structure.D);
    n = nparams(sys.Structure.D, 'free');
    JD = NaN(N, n);
    for i=1:n
        B = -sys.K * der{i};
        D = der{i};
        syss = lpvss(A, B, C, D, ts);
        yhat = lsim(syss, p, u)';
        JD(:, i) = -yhat(:)';
    end

    %% thetaK
    der = parder(sys.Structure.K);
    n = nparams(sys.Structure.K, 'free');
    JK = NaN(N, n);
    for i=1:n
        B = [ - der{i} * sys.C, -der{i} * sys.D, der{i} ];
        D = [];
        syss = lpvss(A, B, C, D, ts);
        yhat = lsim(syss, p, [xhat, u, y])';
        JK(:, i) = -yhat(:)';
    end 

    %% thetaX0
    n = nx;
    JX0 = NaN(N, n);
    for i=1:n
        B = []; D = [];
        syss = lpvss(A, B, C, D, ts);
        x0 = zeros(nx, 1);
        x0(i) = 1;
        yhat = lsim(syss, p, zeros(N, 0), [], x0)';
        JX0(:, i) = -yhat(:)';
    end

    J = [JA, JB, JC, JD, JK, JX0];

    %% Calculate Q (7.17)
    A = sys.A.matrices;
    B = sys.B.matrices;
    C = sys.C.matrices;
    K = sys.K.matrices;

    QA = cell(1, size(A, 3));
    for i=1:size(A, 3)
        QA{i} = kron(A(:, :, i), eye(nx)) - kron(eye(nx), A(:, :, i)');
    end
    QA = cell2mat(QA);

    QB = cell(1, size(B, 3));
    for i=1:size(B, 3)
        QB{i} = kron(B(:, :, i), eye(nx));
    end
    QB = cell2mat(QB);

    QC = cell(1, size(C, 3));
    for i=1:size(C, 3)
        QC{i} = -kron(eye(nx), C(:, :, i)');
    end
    QC = cell2mat(QC);

    QD = zeros(nx^2, size(D, 3) * ny * nu);

    QK = cell(1, size(K, 3));
    for i=1:size(K, 3)
        QK{i} = kron(K(:, :, i), eye(nx));
    end
    QK = cell2mat(QK);

    QX = kron(sys.X0, eye(nx));

    Q = -[QA, QB, QC, QD, QK, QX]';

    %% Calculate P (7.18)
    [Qq, ~] = qr(Q);
    P = Qq(:, nx^2+1:end);

    %% Calculate J_DDLC (7.21)
    J_DDLC = J * P;

    %% Calculate gradient (7.27)
    e_grad = J_DDLC' * e;
 
    [U, S, V] = svd(J_DDLC);  % (7.28)
    s = diag(S);

    %% Compute step direction q
    fully_orthogonal = true;
    while fully_orthogonal
        cond = (s + lambda) >= gamma * (s(1) + lambda);
        n = find(cond, 1, 'last');

        Sn = diag(s(1:n) + lambda ./ s(1:n));  % (7.29)

        % Compute search direction (7.30)
        q = -(V(:, 1:n) / Sn) * U(:, 1:n)' * e;

        % lines 9-10
        if gamma > epsilon
            gamma = max([epsilon, 0.25 * gamma]);
        else
            lambda = max([lambda_min, 2 * lambda]);
        end

        fully_orthogonal = -q' * e_grad < v * norm(e_grad) * norm(q);  % (7.31)
    end
    
    %% Compute step size
    alpha = 2;  % differs from line 12 since we do while loop iteration at least once!
    repeat_alpha = true;
    while repeat_alpha
        alpha = 0.5 * alpha;
        % Try out candidate system using current alpha
        deltap = alpha * P * q;
        sys_candidate = applydeltap(sys, deltap);
        yp_candidate = predict(sys_candidate, data, 1);
        e_candidate = y - yp_candidate;
        repeat_alpha = ...
            norm(e_candidate)^2 > norm(e)^2 + alpha * beta1 * e_grad' * q;
    end
    
    %% Adapt gamma / lambda based on chosen step size
    if alpha >= 1 - eps
        gamma = max([epsilon, 0.25 * gamma]);
        lambda = lambda / 2;
    elseif alpha <= alpha_min
        lambda = max([lambda_min, 2 * lambda]);
    end
    
    %% Update parameter and GUI
    sys = sys_candidate;
    h = update(h, k, norm(e_candidate), norm(deltap));
        
    %% Check termination condition
    doTerminate = true;
    if k > maxIter
        terminate(h, sys, SysIdTermConditions.MaxIterationsReached);
    elseif (e_grad' / (J_DDLC' * J_DDLC + epsilon * eye(size(J_DDLC, 2)))) * e_grad <= epsilon
        terminate(h, sys, SysIdTermConditions.FunctionTolerance);
    else
        doTerminate = false;
    end
end

