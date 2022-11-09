function sysd = c2d(sysc, Ts, method, opts)
%C2D Convert model from continuous to discrete time
%
% Syntax:
%   sysd = c2d(sysc, Ts)
%   sysd = c2d(sysc, Ts, method)
%   sysd = c2d(sysc, Ts, method, opts)
%
% Inputs:
%   sysc: continuous-time LPVLFR object
%   Ts: sampling time in TimeUnit (the sysc.TimeUnit property)
%   method: character vector denoting the conversion method to employ.
%       Options:
%           - 'exact-zoh-euler': the Rectangular (Euler's Forward) Method
%             in the exact zero-order hold (ZOH) setting. See eq. (23) and beyond of 
%             [1] for details.
%           - 'exact-zoh-poly': the Polynomial (Hanselmann) Method
%             in the exact zer-order hold (ZOH) setting. See eq. (27) and
%             beyond of [1] for details. For this method, an additional
%             integer must be specified in the options argument that
%             provides the order of the approximation. Note that this method
%             is a special case of 'exact-zoh-euler' with n = 1.
%           - 'exact-zoh-tustin': the Tusin Method in the exact zero-order hold (ZOH)
%               setting. See eq. (38) of [1] for details.
%
% References:
%   [1] "On the Discretization of Linear Fractional Representations of LPV
%   Systems", R. Toth et. al. (IEEE Transactions on Control Systems
%   Technology), 2012.
%   

%% Input validation
narginchk(2, 4);
if nargin <= 2
    method = 'exact-zoh-euler';
end
assert(any(strcmp(method, {'exact-zoh-euler', 'exact-zoh-poly', 'exact-zoh-tustin'})), ...
    'Discretization method ''%s'' is not supported.', method);
assert(isreal(Ts) && isscalar(Ts) && Ts > 0 && ~isinf(Ts), ...
    'Sampling time ''Ts'' must be a positive scalar');
assert(hasStaticSchedulingDependence(sysc), ...
    '''sysc'' must have static scheduling dependence');
assert(isct(sysc), '''sysc'' must be a continuous-time model');
% Validate options
if strcmp(method, 'exact-zoh-poly')
    if nargin < 4
        opts = 1;  % order of approximation ("n" in (27) in [1])
    end
    assert(isint(opts) && opts >= 1, 'Approximation order must be positive integer');
else
    if nargin >= 4
        error('Options argument is not supported for method ''%s''', method);
    end
end


%% Pre-processing
% Convert Delta to discrete-time
Delta = c2d(sysc.Delta);
% Extract G
G = sysc.G;
% exact-zoh-euler is special case of exact-zoh-poly with n = 1
if strcmp(method, 'exact-zoh-euler')
    method = 'exact-zoh-poly';
    opts = 1;
end

%% Exact ZOH - Polynomial (Hanselmann) Method
% See [1] equation (27) and beyond
if strcmp(method, 'exact-zoh-poly')

% Approximation order ("n" in (27) in [1])
n = opts;

% Construct matrices (28) in [1]

% Pre-compute powers of G.A as they are frequently needed
Apowcache = cell(1, n+1);
Apowcache{1} = eye(size(G.A, 1));
for i=2:n+1
    Apowcache{i} = Apowcache{i-1} * G.A;
end
% Apow(x) returns x-th power of G.A, i.e., G.A^x
Apow = @(x) Apowcache{x+1};

% Construct "A" matrix (top-left block of (28))
Asum = zeros(sysc.Nx);
for l=0:n
    Asum = Asum + Ts^l / (factorial(l)) * Apow(l);
end
Ad = Asum;

% Construct "B1" matrix (top-middle block of (28))
B1 = cell(1, n);
B1{end} = Ts^n / factorial(n) * G.B(:, 1:sysc.Nw);
for l=(n-1):-1:1
    B1{l} = B1{l+1} + Ts^(n-l+1) / factorial(n-l+1) * Apow(n-l);
end
B1 = cell2mat(B1);

% Construct "B2" matrix (top-right block of (28))
B2 = Ts * G.B(:, sysc.Nw+1:end);
for l=2:n
    B2 = B2 + Ts^l / factorial(l) * Apow(l-1) * G.B(:, sysc.Nw+1:end);
end
Bd = [B1, B2];

% Construct "C1" matrix (left-middle block of (28))
C1 = cell(n, 1);
C1{1} = G.C(1:sysc.Nz, :);
for i=2:n
    C1{i} = C1{i-1} * G.A;
end
C1 = cell2mat(C1);

% Construct "C2" matrix (left-bottom block of (28))
C2 = G.C(sysc.Nz+1:end, :);
Cd = [C1; C2];

% Construct "D11" matrix (middle block of (28))
D11 = cell(n, n);
% Initialize to zeros
D11(:) = {zeros(sysc.Nz, sysc.Nw)};
% Fill first column
D11{1, 1} = G.D(1:sysc.Nz, 1:sysc.Nw);
for i=2:n
    D11{i, 1} = G.C(1:sysc.Nz, :) * Apow(i-2) * G.B(:, 1:sysc.Nw);
end
% Fill remaining column from down-shifted copies of left columns
for i=2:n
    D11(i:end, i) = D11(1:(n-i+1), 1);
end
D11 = cell2mat(D11);

% Construct "D12" matrix (right-middle block of (28))
D12 = cell(n, 1);
D12{1} = G.D(1:sysc.Nz, sysc.Nw+1:end);
for i=2:n
    % TODO: what is the right expression here? Dimensions in paper are not
    % correct.
    D12{i} = G.C(1:sysc.Nz, :) * Apow(i-2) * G.B(:, sysc.Nw+1:end);
end
D12 = cell2mat(D12);

% Construct "D21" matrix (bottom-middle block of (28))
D21 = cell(1, n);
D21(:) = {zeros(sysc.Ny, sysc.Nw)};
D21{1} = G.D(sysc.Nz+1:end, 1:sysc.Nw);
D21 = cell2mat(D21);

% Construct "D22" matrix (bottom-right block of (28))
D22 = G.D(sysc.Nz+1:end, sysc.Nw+1:end);

Dd = [D11, D12; D21, D22];

Gd = ss(Ad, Bd, Cd, Dd);
Gd.Ts = Ts;

% Construct Delta (see just below (27))
Deltad = kron(eye(n), Delta);

%% Exact ZOH - Tustin Method
elseif strcmp(method, 'exact-zoh-tustin')

Td = Ts;  % Consistency with paper notation

A = G.A;
B1 = G.B(:, 1:sysc.Nw);
B2 = G.B(:, sysc.Nw+1:end);
C1 = G.C(1:sysc.Nz, :);
C2 = G.C(sysc.Nz+1:end, :);
D11 = G.D(1:sysc.Nz, 1:sysc.Nw);
D12 = G.D(1:sysc.Nz, sysc.Nw+1:end);
D21 = G.D(sysc.Nz+1:end, 1:sysc.Nw);
D22 = G.D(sysc.Nz+1:end, sysc.Nw+1:end);

I = eye(sysc.Nx);
Psi_i = I - (Td / 2) * A;
Ad = (I + (Td / 2) * A) / Psi_i;
B1d = sqrt(Td) * (Psi_i \ B1);
B2d = sqrt(Td) * (Psi_i \ B2);
C1d = sqrt(Td) * (C1 / Psi_i);
C2d = sqrt(Td) * (C2 / Psi_i);
D11d = (Td / 2) * (C1 / Psi_i) * B1 + D11;
D12d = (Td / 2) * (C1 / Psi_i) * B2 + D12;
D21d = (Td / 2) * (C2 / Psi_i) * B1 + D21;
D22d = (Td / 2) * (C2 / Psi_i) * B2 + D22;

Bd = [B1d, B2d];
Cd = [C1d; C2d];
Dd = [D11d, D12d; D21d, D22d];

Gd = ss(Ad, Bd, Cd, Dd);
Gd.Ts = Ts;

% Construct Delta (unchanged)
Deltad = Delta;

end

%% Post-processing
% Assemble DT model
sysd = lpvlfr(Deltad, Gd);
% Copy signal properties (excluding Ts) from sysc to sysd
sysd = LPVcore.copySignalProperties(sysd, sysc);

end

