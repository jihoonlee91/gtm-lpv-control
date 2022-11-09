function sys = lpvpbsidopt_(data, sys_template, options, gui)
%LPVPBSIDOPT_ LPV Subspace Identification based on LPV-PBSIDopt
%
%   Reference: [1] "Subspace identification of Bilinear and LPV systems for
%       open- and closed-loop data" (J-W van Wingerden, M. Verhaegen, 2009)
%
%   Additional derivations: [2] "LPVPBSID_opt additional documentation",
%       www.lpvcore.net
%
%   Syntax:
%       sys = lpvpbsidopt_(data, sys_template, p)
%
%   Inputs:
%       data: lpviddata containing the measured IO data
%       sys_template: template LPVSS system from which the scheduling
%           dependence and state dimension is extracted.
%       options: lpvsidOptions options set
%       gui: progressgui object
%
%   See also LPVSID, LPVSIDOPTIONS

%% NOTE: "p" here means past window size, in line with the notation in the paper

%% Input validation
narginchk(4, 4);
assert(isa(sys_template, 'lpvidss'), '''sys_template'' must be an LPVIDSS object');
assert(strcmp(typeNoiseModel(sys_template), 'innovation'), ...
    '''sys_template'' must have innovation noise model');
assert(sys_template.Ts ~= 0, '''sys_template'' must be discrete-time');
assert(isa(options, 'lpvsidOptions'), '''options'' must be an lpvsidOptions object');
% Extract system matrices
A = sys_template.A;
B = sys_template.B;
C = sys_template.C;
D = sys_template.D;
K = sys_template.K;
% First basis function must be pbconst
assert(isa(A.bfuncs{1}, 'pbconst'), 'First basis function of A must be "1"');
assert(isa(B.bfuncs{1}, 'pbconst'), 'First basis function of B must be "1"');
assert(isa(C.bfuncs{1}, 'pbconst'), 'First basis function of C must be "1"');
assert(isa(D.bfuncs{1}, 'pbconst'), 'First basis function of D must be "1"');
assert(isa(K.bfuncs{1}, 'pbconst'), 'First basis function of K must be "1"');

% Following paper notation, l is output size, r is input size
l = size(C, 1);
r = size(B, 2);
n = size(A, 1);

% Past window size
p = options.PastWindowSize;
kernelization = options.Kernelization;
lambda = options.KernelizationRegularizationConstant;

assert(p * l > n, ...
    'p * l > n not satisfied (p: past window size, l: output size, n: state size');

progressmsg(gui, 'Identification using LPV-PBSIDopt');
progressmsg(gui, 'The numbers refer to equations in the reference (J-W van Wingerden, M. Verhaegen, 2009)');
drawnow;

%% 1. Create data matrices

% Calculate closed-loop template matrices (for the basis function
% evaluation) (At = \tilde{A}, Bt = \tilde{B} in paper).
% SIMPLIFY is used here to ensure the constant basis function is the first.
At = simplify(A - K * C);
Bt = simplify(B - K * D);
% Sanity check to ensure the closed-loop template matrices have the first
% basis function constant
assert(isa(At.bfuncs{1}, 'pbconst'), ...
    'First basis function of At is not "1"');
assert(isa(Bt.bfuncs{1}, 'pbconst'), ...
    'First basis function of Bt is not "1"');

% Evaluate the closed-loop and open-loop basis functions
% "idx" holds the remaining row indices after truncation
[psiAt, psiBt, psiA, psiB, psiC, psiD, psiK, ~, idx] = ...
    pevalmulti(At, Bt, A, B, C, D, K, data.p, (1:size(data.p, 1))');

% Truncate input-output data to account for dynamic dependency
datay = data.y(idx, :);
datau = data.u(idx, :);
N = size(datay, 1);

muAt = psiAt.'; muBt = psiBt.';
muA = psiA.'; muB = psiB.';
muC = psiC.'; muD = psiD.';
muK = psiK.';

% Get basis dimension for closed- and open-loop matrices
mAt = size(muAt, 1);
mBt = size(muBt, 1);
mA = size(muA, 1); mB = size(muB, 1);
mC = size(muC, 1); mD = size(muD, 1);
mK = size(muK, 1);

% Check persistence of excitation
assert(rank(muAt) == mAt, 'Scheduling signal not persistently exciting');
assert(rank(muBt) == mBt, 'Scheduling signal not persistently exciting');
assert(rank(muC) == mC, 'Scheduling signal not persistently exciting');
% A, B, D, K do not need to be included in this condition since their basis
% functions are a subset of At and Bt.
assert(N - p + 1 > max([mAt, mBt, mC]), 'Not enough IO data provided');

% Note that the dimension of Kp in the paper is slightly different for us,
% since we allow different basis functions of Bt and At
progressmsg(gui, '(Definition 5) Constructing Pkp');
qt = (r + l) * sum(mBt * mAt.^((1:p) - 1));  % \tilde{q}
Pkp = cell(N, p);
for k=1:N
    % Note we start with mu of Bt
    R = kron(muBt(:, k), eye(r+l));
    Pkp{k, 1} = R;
    for j=2:p
        if k + j - 1 <= N
            % Recurrence
            R = kron(muAt(:, k+j-1), R);
            Pkp{k, j} = R;
        else
            % Move to next time sample
            break;
        end
    end
end
% TODO: remove these checks
assert(all(size(Pkp{1, p}) == [mBt * mAt^(p-1) * (r+l), r+l]));

% Relation between Qpk and Ppk:
%   Q_{p}^{k} = muC_{k+p} \kron P_{p}^{k}
progressmsg(gui, '(Definition 5) Constructing Qkp');
Qkp = cell(N, p);
for k=1:N
    for j=1:p
        if k + j <= N
            Qkp{k, j} = kron(muC(:, k+j), Pkp{k, j});
        else
            Qkp{k, j} = NaN;
        end
    end
end
% TODO: remove these checks
assert(all(size(Qkp{1, p}) == [mC*mAt^(p-1)*mBt * (r+l), r+l]));
assert(all(size(Qkp{N-p, p}) == [mC*mAt^(p-1)*mBt * (r+l), r+l]));

% Construct (7)
progressmsg(gui, '(7) Constructing Nkp');
Nkp = cell(1, N-p);
% Iterate over the time stamps
for k=1:N-p+1
    % Iterate over the block diagonal elements of N
    Nkp{k} = [];
    for i=0:p-1
        Nkp{k} = blkdiag(Nkp{k}, Pkp{k+i, p-i});
    end
    % TODO: remove this check
    assert(all(size(Nkp{k}) == [qt, p * (r + l)]));
end

% Construct Mkp
progressmsg(gui, '(7) Constructing Mkp');
Mkp = cell(1, N-p);
% Iterate over the time stamps
for k=1:N-p
    % Iterate over the block diagonal elements of N
    Mkp{k} = [];
    for i=0:p-1
        Mkp{k} = blkdiag(Mkp{k}, Qkp{k+i, p-i});
    end
    % TODO: remove this check
    assert(all(size(Mkp{k}) == [mC * qt, p * (r + l)]));
end

% IO data matrices (10)-(12)
U = datau(p+1:N, :).';
Y = datay(p+1:N, :).';
z = [ datau, datay ].';

% Each column of zbar contains a past window of z (see just below (5))
progressmsg(gui, '(Below (5)) Constructing zbar');
zbar = NaN((r + l) * p, N - p);
for i=1:p
    zbar(((i-1)*(r+l)+1):(i*(r+l)), :) = z(:, i:(i+N-p-1));
end

%% Construct Z (12) and ZM
% Z (12) for calculation of the state sequence
progressmsg(gui, '(12) Constructing Z');
Z = NaN(qt, N-p);
for k=1:N-p
    Z(:, k) = Nkp{k} * zbar(:, k);
end

% ZM for finding Hp
ZH = NaN(mC * qt, N-p);
for k=1:N-p
    ZH(:, k) = Mkp{k} * zbar(:, k);
end

% UM for finding D
UM = kron(muD(:, p+1:N), ones(r, 1)) .* repmat(U, mD, 1);

%% Solve optimization problem (13)
if ~kernelization
    progressmsg(gui, '(13) Solving for ext. observability matrix CKp');
    Hp_D = Y / [ ZH; UM ];
    % Extract Hp and D
    Hp = Hp_D(:, 1:mC*qt);
    M_D = Hp_D(:, mC*qt+1:end);
    
    % TODO: remove these checks
    assert(all(size(Hp) == [l, mC*qt]));
    assert(all(size(M_D) == [l, mD*r]));
    
    %% Extract CKp from Hp
    % Only the columns of Hp that correspond to the parameter-independent part
    % of C are needed.
    CKp = [];
    colidx = 1;  % column index as it scans over Hp
    % Loop over Ip
    for i=p:-1:1
        % The width of Li within Ii
        Li_width = (r+l)*mBt*mAt^(i-1);
        CKp = [CKp, Hp(:, colidx:colidx+Li_width-1)]; %#ok<AGROW>
        % Of the "mC" blocks, only the first one corresponds to
        % parameter-independent part of C. Thus, we skip the remaining "mC-1".
        colidx = colidx + mC * Li_width;
    end
    
    %% Construct GpKp (14)
    % Note: Dimension of Lj is n-by-(r+l)mBt*mAt^(j-1)
    
    % GpKp is constructed row-by-row (l rows at a time, to be exact) 
    % by copying certains blocks of columns from a previous block row.
    % For example, the first columns of CLp, CLp-1, ..., CL2 of the first row
    % in (14) appear again in the second row (by the relation specified below).
    % The variable blkcol keeps track of the indices of these columns that are
    % re-used. There are p blocks.
    progressmsg(gui, '(14) Constructing GpKp');
    blkcol = cell(1, p);
    start = 1;
    % Iterate over the p block columns in (14)
    for i=1:p
        % The number of columns in this block that are re-used, which is
        % the first "one-mAt'th" of CLi
        d = (r+l)*mAt^(p-i-1)*mBt;
        % The corresponding columns
        blkcol{i} = start:start+d-1;
        % Shift start to next block column
        start = start + (r+l)*mAt^(p-i)*mBt;
    end
    
    GpKp = [ CKp; zeros(l*(p-1), qt) ];
    % Iterate over the block rows
    for i=2:p
        % Row indices of this block row
        rowidx = ((i-1)*l+1:i*l);
        % Non-zero column indices associated with this block row
        coldim = qt;
        colnzdim = (r+l)*sum(mBt*mAt.^((1:(p-i+1))-1));
        colidx = (coldim-colnzdim+1):coldim;
        % Copy the block columns from the previous block row
        newrow = [];
        for j=i-1:p-1
            % Idx of previous row is rowidx - l
            newrow = [newrow, GpKp(rowidx-l, blkcol{j})]; %#ok<AGROW>
        end
        % Assign the new row to GpKp
        GpKp(rowidx, colidx) = newrow;
    end
end

% Alternatively, construct the matrix JHp that selects the appropriate rows of Hp
% See (7) in [2]

JHp = [];
% TODO: make this section neater
JHp_blocks = cell(1, p);
for i=p:-1:1
    JHp_blocks{i} = kron([1; zeros(mC-1)], eye((r+l)*mBt*mAt^(i-1)));
    JHp = blkdiag(JHp, ...
        JHp_blocks{i});
end
% CKp_alt = Hp * JHp;
% TODO: remove this check
% assert(norm(CKp - CKp_alt) / norm(CKp) < 1E-15);

%% Kernelization method
% Note: Nbar is different from paper, they made a dimension mistake in (12)
if kernelization
    progressmsg(gui, '(23) Calculating Zij and ZHij');
    Nbar = N - p;
    Zij = cell(p, p);
    ZHij = cell(p, p);
    for i=1:p
        for j=i:p
            % Each Zij and ZHij in (23) has Nbar columns
            % Each Zij / ZHij in (23) has as many rows as Pkp / Qkp
            Zij_ = NaN(mBt*mAt^(p-j)*(r+l), Nbar);
            ZHij_ = NaN(mC*mBt*mAt^(p-j)*(r+l), Nbar);
            for k=1:Nbar
                Zij_(:, k) = Pkp{k+j-i, p-j+1} * z(:, j-i+k);
                ZHij_(:, k) = Qkp{k+j-i, p-j+1} * z(:, j-i+k);
            end
            Zij{i, j} = Zij_;
            ZHij{i, j} = ZHij_;
        end
    end
    
    % Check Zij and ZHij using Theorem 10
    
    % Important: to debug kernelization, ZHij_sum can be compared to Zij_sum.
    % To do this, uncomment the lines starting with Zij_sum and also the
    % asserts after the for-loop. However, this will be very slow!
    
    % Zij_sum = zeros(Nbar);
    ZHij_sum = zeros(Nbar);
    for j=1:p
        % Zij_sum = Zij_sum + Zij{1, j}.' * Zij{1, j};
        ZHij_sum = ZHij_sum + ZHij{1, j}.' * ZHij{1, j};
    end
    % Uncomment this check to debug kernelization
    % assert(norm(Zij_sum - Z.' * Z) / norm(Z.' * Z) < 1E-15);
    % assert(norm(ZHij_sum - ZH.' * ZH) / norm(ZH.' * ZH) < 1E-15);
    
    % Calculate alpha with Tikhonov regularization
    % Eq. (20) in reference
    progressmsg(gui, '(20) Calculating alpha with Tikhonov regularization');
    alpha = Y / (ZHij_sum + U.' * U + lambda * eye(Nbar));
    
    % Estimate of GpKpZ using the kernel method
    GpKpZ_kernel = cell(p, 1);
    for i=1:p
        GpKpZ_kernel{i} = ZHij{i, i}.' * JHp_blocks{p-i+1} * Zij{1, i};
        for j=i+1:p
            GpKpZ_kernel{i} = GpKpZ_kernel{i} + ...
                ZHij{i, j}.' * JHp_blocks{p-j+1} * Zij{1, j};
        end
        GpKpZ_kernel{i} = alpha * GpKpZ_kernel{i};
    end
    GpKpZ_kernel = cell2mat(GpKpZ_kernel);
end

%% Estimate state sequence (15)-(16)
progressmsg(gui, '(15)-(16) Estimating state sequence using SVD');
if kernelization
    warning('Kernelization enabled: this is a preview feature');
    [~, Sigma, V] = svd(GpKpZ_kernel(1:n, :));
else
    GpKpZ = GpKp * Z;
    [~, Sigma, V] = svd(GpKpZ(1:n, :));
end
% Select first n singular values and columns of V
Xhat = sqrt(Sigma) * V';  % (16)

%% Obtain the system matrices (1)-(2)
progressmsg(gui, '(1)-(2) Estimating system matrices based on estimated state sequence');
% First, calculate M_C and M_D using (2)
XM = kron(muC(:, 1+p:N), ones(n, 1)) .* repmat(Xhat, mC, 1);
W = Y / [XM; UM];
M_C = W(:, 1:size(XM, 1));
M_D = W(:, size(XM,1)+1:end);

% Then, estimate the noise E and calculate A, B, K
E = Y - M_C * XM - M_D * UM;
S = NaN(mA*n + mB*r + mK*l, N-p-1);
for k=1:N-p-1
    S(:, k) = [kron(muA(:, k+p), Xhat(:, k)); ...
        kron(muB(:, k+p), U(:, k)); ...
        kron(muK(:, k+p), E(:, k))];
end
M_ABK = Xhat(:, 2:end) / S;

% Extract matrix representations of each separate matrix A, B, and K
M_A = M_ABK(:, 1:n*mA);
M_B = M_ABK(:, n*mA+1:n*mA+r*mB);
M_K = M_ABK(:, n*mA+r*mB+1:end);

A = reshape(M_A, n, n, mA);
B = reshape(M_B, n, r, mB);
C = reshape(M_C, l, n, mC);
D = reshape(M_D, l, r, mK);
K = reshape(M_K, n, l, mK);

sys = sys_template;
sys.A.matrices = A;
sys.B.matrices = B;
sys.C.matrices = C;
sys.D.matrices = D;
sys.K.matrices = K;

%% Show results
terminate(gui, sys, SysIdTermConditions.NonIterative);

end

