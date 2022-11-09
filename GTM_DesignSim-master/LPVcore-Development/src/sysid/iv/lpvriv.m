function [sys, pvec, theta] = lpvriv(data, template_sys, options)
%LPVRIV Identification of LPV-BJ model based refined instrumental variables
%
%   Syntax:
%       sys = lpvriv(data, template_sys)
%       sys = lpvriv(data, template_sys, options)
%       [sys, pvec, theta] = lpvriv(___)
%
%   Inputs:
%       data: lpviddata object
%       template_sys: template lpvidpoly object.
%       options: lpvrivOptions object.
%
%   Outputs:
%       sys: estimated system model.
%       pvec: column vector of parameters of sys oriented according to
%           LPVcore standard: B, C, D, F polynomials in that order.
%       theta: column vector of parameters of sys oriented according to
%           notation in [1]: F, B, C, D polynomials in that order.
%
%   Reference:
%       [1] Algorithm 2 from "Refined instrumental variable methods for
%       identification of LPV Box-Jenkins models" in Automatica 46 (2010)
%       by V. Laurain et. al.
%
%   See also LPVRIVOPTIONS
%

%
% Additional documentation to paper [1].
%
% Size of vectors in (38):
%   yf: same size as y
%   phi_f: see just below (34)

% TODO: add a flag "EnforceStability" that stabilizes the scheduling
% independent part of the F polynomial. By default it is true.

% TODO: add an option for armaxOptions for identifying the noise part.

% TODO: add an option for arxOptions for the initialization.

%% Input validation
narginchk(2, 3);
assert(isa(data, 'lpviddata'), '''data'' must be an lpviddata object');
y = data.y;
ny = size(y, 2);
u = data.u;
nu = size(u, 2);
p = data.p;
assert(isa(template_sys, 'lpvidpoly') && ...
    isbj(template_sys), '''template_sys'' must be an lpvidpoly object representing an LPV-BJ model structure');
if nargin <= 2
    options = lpvrivOptions;
end
assert(isa(options, 'lpvrivOptions'), '''options'' must be an lpvrivOptions object');

% Check if noise part is scheduling-independent
assert(isconst(template_sys.C) && isconst(template_sys.D), ...
    '''template_sys'' must have scheduling-independent noise part (C and D)');
% Check if first basis function of each coefficient in F is
% scheduling-independent
for i=1:template_sys.Nf +1
    F_ = template_sys.F.Mat{i};
    assert(isa(F_.bfuncs{1}, 'pbconst'), 'First basis function of each coefficient in F must be constant');
end
% Check if basis functions of all coefficients in F after the first 1 are
% identical (assumed by (5))
for i=2:template_sys.Nf + 1
    F_ = template_sys.F.Mat{i};
    if i > 2
        Fprev_ = template_sys.F.Mat{i-1};
        assert(isequal(F_.bfuncs, Fprev_.bfuncs), ...
            'Basis functions of each coefficient (except 1st one) in F must be identical');
    end
end
% Check if F is diagonal (MIMO systems are treated as a set of ny MISO
% systems).
assert(isdiag(template_sys.F), ...
    'Off-diagonal elements of F of ''template_sys'' must be 0 and non-free');

% Create GUI if Display is on
gui = progressgui(template_sys, data, options);

%% Algorithm

% Step 1: initialization
init = options.Initialization;
if strcmp(init, 'polypre')
    progressmsg(gui, 'Initialization using LPV-ARX estimate');
    % Define ARX template system
    template_sys_arx = lpvidpoly(template_sys.F, template_sys.B);
    % Solve LPVARX identification
    sys_arx = lpvarx(data, template_sys_arx);
    % Copy ARX results to BJ model structure to use as initialization
    sys = template_sys;
    sys.F = sys_arx.A;
    sys.B = sys_arx.B;
    [~, fit] = compare(data, sys);
    progressmsg(gui, ...
        sprintf('Initialization using LPV-ARX estimate finished. Fit: [%s]%%\n', num2str(fit')));
elseif strcmp(init, 'template')
    progressmsg(gui, 'Initialization from provided template model');
    sys = template_sys;
end

% Order of B
nb = sys.Nb;
% Number of nonconstant basis functions
nbeta = size(sys.B.Mat{1}, 3) - 1;

% Order of F (A in paper notation [1])
na = sys.Nf;
% Number of nonconstant basis functions (see just below (5))
nalpha = size(sys.F.Mat{2}, 3) - 1;

% Maximum order of B and F
n = max([na, nb]);

% Loop over output channels
sys_per_channel = cell(1, ny);
for output_idx=1:ny
    output_name = sys.OutputName{output_idx};
    progressmsg(gui, '');
    if isempty(output_name)
        progressmsg(gui, sprintf('RIV identification for output channel %i', output_idx));
    else
        progressmsg(gui, sprintf('RIV identification for output channel %i (''%s'')', ...
            output_idx, output_name));
    end
    progressmsg(gui, '');
    sysi = sys(output_idx, :);
    yi = y(:, output_idx);
    % Loop over iterations for the output channel
    for iter=1:options.MaxIterations
        % Step 2: estimate noise-free output
        [~, ~, ~, chihat] = lsim(sysi, data.p, data.u);
        % Multiplication of noise-free output with each basis function of F
        % Each row corresponds to a time step:
        %   
        %   Chihat = [chihat1; chihat2; ..., chihatn]
        %
        %   chihat1 = [chihat_{1,1}, ..., chihat_{na, nalpha}] at time k = 1
        %
        Chihat = NaN(size(chihat, 1)-n, na * nalpha);
        % Iterate over time steps
        for k=1:size(chihat, 1)-n
            chihati = NaN(na, nalpha);
            % Iterate over coefficients
            for i=1:na
                % Iterate over basis functions
                for l=1:nalpha
                    fl = sys.F.Mat{i+1}.bfuncs{l+1};
                    chihati(i, l) = feval(fl, p(k+n, :)) * chihat(k + n - i, :);
                end
            end
            Chihat(k, :) = reshape(chihati, 1, na * nalpha);
        end
    
        % Multiplication of input u with each basis function of B as in (32)
        % Each row corresponds to a time step:
        %   
        %   U = [u1; u2; ...; un]
        %
        %   u1 = [u_{1,1}, ..., u_{na, nalpha}] at time k = 1
        %
        U = NaN(size(chihat, 1)-n, (nb + 1) * (nbeta + 1) * nu);
        % Iterate over time steps
        for k=1:size(chihat, 1)-n
            uj = NaN(nu, nb + 1, nbeta + 1);
            % Iterate over coefficients
            for j=0:nb
                % Iterate over basis functions
                for l=0:nbeta
                    fl = sys.B.Mat{j+1}.bfuncs{l+1};
                    uj(:, j+1, l+1) = feval(fl, p(k+n, :)) * u(k + n - j, :);
                end
            end
            U(k, :) = reshape(uj, 1, (nb + 1) * (nbeta + 1) * nu);
        end
    
        % Step 3: compute the estimated filter Q
        % Q is computed as a filter representation:
        % yf = filter(Qb, Qa, y)
        [Qb, Qa] = compute_Q(sysi);
        yf = filter(Qb, Qa, yi);
        u_jl_f = filter(Qb, Qa, U, [], 1);
        chihat_f = filter(Qb, Qa, chihat);
        chihat_il_f = filter(Qb, Qa, Chihat, [], 1);  % filter along 1st dimension (filter applied to each column)
    
        % Step 4: build the regressor phi and filter instrument zeta
        N = size(yf, 1) - n;
        Phi_f = NaN(na + na * nalpha + (nb + 1) * (nbeta + 1) * nu, N);
        for k=1:N
            Phi_f(:, k) = [-yf(k:k+n-1, :); -chihat_il_f(k, :).'; u_jl_f(k, :).'];
        end
    
        Psi_f = NaN(na + na * nalpha + (nb + 1) * (nbeta + 1) * nu, N);
        for k=1:N
            Psi_f(:, k) = [-chihat_f(k:k+n-1, :); -chihat_il_f(k, :).'; u_jl_f(k, :).'];
        end
    
        Y_f = yf(n+1:N+n, :)';
    
        % Step 5: solve the IV optimization problem
        rho = (Psi_f * Phi_f') \ (Psi_f * Y_f'); % (40)
    
        % Step 6: estimate noise part C, D
        vhat = yi - chihat;
        sysi_noise = armax(iddata(vhat, []), ...
            [template_sys.Nd, template_sys.Nc], ...
            armaxOptions('ProgressWindow', 'keep'));  % Needed to keep progress window open
        % Get parameter vector eta from sys_noise
        eta = getpvec(sysi_noise);
    
        % Step 7: check convergence
        theta = [rho; eta];
        if iter > 1
            step = norm(theta - theta_prev);
        else
            step = 0;
        end
        % Store theta to calculate step size in next iteration
        theta_prev = theta;
    
        % Update model parameters
        pvec = theta_to_pvec(sysi, theta);
        sysi = setpvec(sysi, pvec);

        % Calculate cost: 2-norm of output error
        datai = lpviddata(yi, p, u, template_sys.Ts);
        yi_hat = compare(datai, sysi);
        cost = norm(yi_hat - yi) / size(yi, 1);
    
        % Update progress GUI
        update(gui, iter, cost, step);
    end
    sys_per_channel{output_idx} = sysi;
end

% Concatenate sys_per_channel into sys
sys = sys_per_channel{1};
for i=2:ny
    sys = [sys; sys_per_channel{i}]; %#ok<AGROW> 
end

terminate(gui, sys, SysIdTermConditions.MaxIterationsReached);

end

%% Local functions

function p = theta_to_pvec(sys, theta)
    % THETA2PVEC Convert theta from paper notation to parameter vector in
    % LPVcore.
    %
    % theta can be split up as follows:
    %   theta = [
    %       rho;    <-- process parameters
    %       eta     <-- noise parameters
    %   ]
    %
    %   rho = [
    %       a;      <-- A-polynomial parameters (sys.F in LPVcore)
    %                   with na * nalpha values
    %       b       <-- B-polynomial parameters (also sys.B in LPVcore)
    %                   with (nb + 1) * (nbeta + 1) * nu values
    %   ]
    %
    %   eta = [
    %       c;      <-- C-polynomial parameters (also sys.C in LPVcore)
    %                   with nc values
    %       d;      <-- D-polynomial parameters (also sys.D in LPVcore)
    %                   with nd values
    %   ]
    %       
    
    nu = sys.Nu;
    nb = sys.Nb;
    nbeta = size(sys.B.Mat{1}, 3) - 1;
    na = sys.Nf;
    nalpha = size(sys.F.Mat{2}, 3) - 1;
    nc = sys.Nc;
    nd = sys.Nd;

    nrho = na * (nalpha + 1) + (nb + 1) * (nbeta + 1) * nu;
    neta = nc + nd;

    assert(numel(theta) == nrho + neta, ...
        '''theta'' has %i parameters but must have nrho (%i) + neta (%i) = %i', ...
        numel(theta), nrho, neta, nrho + neta);
    rho = theta(1:nrho);
    eta = theta(nrho+1:end);
    a = rho(1:na*(nalpha+1));
    b = rho(na*(nalpha+1)+1:end);
    
    % Order in LPVcore:   A , B, C, D , F   polynomials in LPVcore notation,
    % corresponding to    [], b, [eta], a   in paper notation
    p = [b; eta; a];
end

function [F_constant, F_nonconstant] = split_F(F)
    % SPLIT_F Split F into scheduling independent and scheduling dependent
    % parts
    % Assume first basis function of each coefficient is constant
    Nf = F.N;
    mat_constant = cell(1, Nf); mat_nonconstant = cell(1, Nf);
    free_constant = cell(1, Nf); free_nonconstant = cell(1, Nf);
    min_constant = cell(1, Nf); min_nonconstant = cell(1, Nf);
    max_constant = cell(1, Nf); max_nonconstant = cell(1, Nf);
    for i=1:Nf
        % Get properties of F
        mat = F.Mat{i};
        free = F.Free{i};
        min = F.Min{i};
        max = F.Max{i};
        % Split properties of F into constant and non-constant
        mat_constant{i} = slice(mat, 1);
        free_constant{i} = free(:, :, 1);
        min_constant{i} = min(:, :, 1);
        max_constant{i} = max(:, :, 1);
        if size(mat, 3) > 1
            mat_nonconstant{i} = slice(mat, 2, 'end');
            free_nonconstant{i} = free(:, :, 2:end);
            min_nonconstant{i} = min(:, :, 2:end);
            max_nonconstant{i} = max(:, :, 2:end);
        else
            mat_nonconstant{i} = 0 * mat(:, :, 1);
            free_nonconstant{i} = 0 * free(:, :, 1);
            min_nonconstant{i} = -Inf * min(:, :, 1);
            max_nonconstant{i} = Inf * max(:, :, 1);
        end
    end
    F_constant = pidpoly(mat_constant, free_constant, min_constant, max_constant);
    F_nonconstant = pidpoly(mat_nonconstant, free_nonconstant, min_nonconstant, max_nonconstant);
end

function [Qb, Qa] = compute_Q(sys)
    % NOTE: "F" in paper is NOT "sys.F"!
    % "F" refers to the scheduling-independent part of "sys.F".
    % Q = D / (C * F)       (eq. 37)
    % Consistent with MATLAB FILTER command,
    % Q is decomposed as Qb / Qa.
    % Thus, Qb = D and Qa = C * F

    % Extract LTI part of F (assumed to be the first matrix of each PMATRIX coefficient).
    Flti = extract_Flti(sys);
    % C and D are assumed to be scheduling-independent, so we can use
    % FREEZE.
    Ccell = freeze(sys.C, zeros(1, sys.Np));
    Clti = Ccell{1};
    Dcell = freeze(sys.D, zeros(1, sys.Np));
    Dlti = Dcell{1};
    % Define polynomials
    Qb = Dlti;
    Qa = conv(Clti, Flti);
end

function signal_filtered = apply_Q(D, C, F_constant, signal)
    % TODO: remove this function (support for full MIMO systems is dropped)
    % NOTE: "F" in paper is NOT "sys.F"!
    % "F" refers to the scheduling-independent part of "sys.F".
    % Q = (F * C) \ D
    %
    %   Action of Q on signal s:
    %                 sf = (Q) * s
    %       (F * C) * sf = (D) * s
    %       (F) * C * sf = (D) * s
    %             ------
    %               |
    %              shat
    %
    %       (F) * shat = (D) * s
    %       (C) * sf   = shat
    %

    sys_FD = lpvio(F_constant.Mat, D.Mat);
    sys_C = lpvio(C.Mat, eye(size(C, 1)));

    N = size(signal, 1);
    shat = lsim(sys_FD, NaN(N, sys_FD.Np), signal);
    signal_filtered = lsim(sys_C, NaN(N, sys_C.Np), shat);
end

function Flti = extract_Flti(sys)
    % EXTRACT_FLTI Extract Flti, the scheduling independent part of F, from model defined by (32)
    Flti = NaN(1, sys.Nf + 1);
    for i=1:sys.Nf+1
        Flti(i) = sys.F.Mat{i}.matrices(:, :, 1);
    end
end

