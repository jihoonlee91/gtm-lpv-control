classdef (InferiorClasses = {?ss, ?tf, ?zpk}) lpvss < lpvlfr
%LPVSS LPV State-Space system representation
%
%   Syntax:
%           SYS = lpvss(A,B,C,D) creates an object SYS representing the
%      continuous-time state-space model
%           dx/dt = A(p(t)) x(t) + B(p(t)) u(t)
%            y(t) = C(p(t)) x(t) + D(p(t)) u(t)
%      The A, B, C, D matrices can be real matrices or pmatrix
%      objects. For D = [], the matrix will be set to a zero matrix of
%      appropriate dimension.
%  
%      SYS = lpvss(A,B,C,D,Ts) creates a discrete-time state-space model with
%      sample time Ts (set Ts=-1 if the sample time is undetermined) of the
%      following form:
%          x(t+1) = A(p(t)) x(t) + B(p(t)) u(t)
%            y(t) = C(p(t)) x(t) + D(p(t)) u(t)
%  
%      SYS = lpvss(D) specifies a static gain matrix D.
%
%      SYS = lpvss(A,B,C,D,Ts,'Property1',Value1,...,'PropertyN',ValueN)
%      will set the property value pairs. In addition to all properties
%      supported by LPVLFR, the following properties are available:
%
%           'SVDTolerance': sets the SVDTolerance property of the LPVSS
%               object.
%   
%   See also LPVIO, LPVIDPOLY, PMATRIX, TIMEMAP.
    
    properties
        % (Advanced) Tolerance for discarding small singular values
        %
        % LPVSS models are internally represented as LPVLFR models. In the
        % conversion process, a singular value decomposition is employed.
        % Without a tolerance on the singular values, the Delta block of
        % the corresponding LPVLFR representation grows quickly. All
        % singular vectors below this tolerance are discared to remedy
        % this. The default value is sqrt(eps).
        SVDTolerance
    end

    properties (Dependent)
        A       % nx-by-nx pmatrix or empty object
        B       % nx-by-nu pmatrix or empty object
        C       % ny-by-nx pmatrix or empty object
        D       % ny-by-nu pmatrix or empty object
    end
    
    methods
        function obj = lpvss(A, B, C, D, Ts, varargin)
            if nargin == 1
                if isnumeric(A) || isa(A, 'ss') || isa(A, 'tf') || isa(A, 'zpk')
                    sys = ss(A, 'minimal');
                    A = sys.A;
                    B = sys.B;
                    C = sys.C;
                    D = sys.D;
                    Ts = sys.Ts;
                elseif isa(A, 'lpvss')
                    sys = A;
                    A = sys.A;
                    B = sys.B;
                    C = sys.C;
                    D = sys.D;
                    Ts = sys.Ts;
                elseif isa(A, 'lpvlfr')
                    sys = A;
                    [A, B, C, D, Ts] = lpvlfr2ss_(sys);
                end
            end

            % Ensure compatible sizes of A, B, C, D
            if size(A, 1) ~= size(A, 2)
                error('The "A" matrix must be square');
            end
            nx = size(A, 1);
            % Infer IO dimensions
            if ~isempty(D)
                nu = size(D, 2);
                ny = size(D, 1);
            else
                nu = size(B, 2);
                ny = size(C, 1);
            end
            % Expand empty matrices to appropriate IO dimensions
            if isempty(B)
                B = zeros(nx, nu);
            end
            if isempty(C)
                C = zeros(ny, nx);
            end
            if isempty(D)
                D = zeros(ny, nu);
            end
            % Check compatibility of B, C, D with IO
            if size(B, 2) ~= nu || size(D, 2) ~= nu
                error('The "B" and "D" matrices must have the same number of columns.');
            end
            if size(C, 1) ~= ny || size(D, 1) ~= ny
                error('The "C" and "D" matrices must have the same number of rows.');
            end
            % Check compatibility of B, C with state
            if size(B, 1) ~= nx
                error('The "A"and "B" matrices must have the same number of rows.');
            end
            if size(C, 2) ~= nx
                error('The "A"and "C" matrices must have the same number of columns.');
            end
            
            % Convert to PMATRIX
            if isa(A, 'double'); A = double2pmatrix(A); end
            if isa(B, 'double'); B = double2pmatrix(B); end
            if isa(C, 'double'); C = double2pmatrix(C); end
            if isempty(D)
                D = pzeros(size(C, 1), size(B, 2));
            end
            if isa(D, 'double'); D = double2pmatrix(D); end
            
            [A, B, C, D, tm] = ...
                pmatrix.commonrho_(A, B, C, D);
            if nargin >= 3 && nargin <= 4
                if any(strcmp(tm.Domain, {'undefined', 'ct'}))
                    Ts = 0;
                else
                    Ts = -1;
                end
            end
            
            % Parse PV pairs
            parser = inputParser;
            parser.KeepUnmatched = true;
            addParameter(parser, 'SVDTolerance', sqrt(eps));
            parse(parser, varargin{:});
            svdtol = parser.Results.SVDTolerance;

            [Delta, G] = lpvss2lfr_(A, B, C, D, Ts, svdtol);
            obj = obj@lpvlfr(Delta, G, varargin{:});
            % If 'sys' is defined, inherit signal properties from it
            if exist('sys', 'var')
                obj = LPVcore.copySignalProperties(obj, sys, 'full');
            end
            % Add SVD tolerance
            obj.SVDTolerance = svdtol;
        end
        
        function val = get.A(obj)
            val = obj.A0 + obj.Bw * obj.Delta * obj.Cz;
        end
        
        function val = get.B(obj)
            val = obj.Bu + obj.Bw * obj.Delta * obj.Dzu;
        end
        
        function val = get.C(obj)
            val = obj.Cy + obj.Dyw * obj.Delta * obj.Cz;
        end
        
        function val = get.D(obj)
            val = obj.Dyu + obj.Dyw * obj.Delta * obj.Dzu;
        end
        
        function obj = set.A(obj, A)
            if ~all(size(A) == [obj.Nx, obj.Nx])
                error('A must be Nx-by-Nx');
            end
            [obj.Delta_, obj.G_] = lpvss2lfr_(A, obj.B, obj.C, obj.D, obj.Ts, obj.SVDTolerance);
        end
        
        function obj = set.B(obj, B)
            if isempty(B); B = zeros(obj.Nx, obj.Nu); end
            if ~all(size(B) == [obj.Nx, obj.Nu])
                error('B must be Nx-by-Nu');
            end
            [obj.Delta_, obj.G_] = lpvss2lfr_(obj.A, B, obj.C, obj.D, obj.Ts, obj.SVDTolerance);
        end
        
        function obj = set.C(obj, C)
            if isempty(C); C = zeros(obj.Ny, obj.Nx); end
            if ~all(size(C) == [obj.Ny, obj.Nx])
                error('C must be Ny-by-Nx');
            end
            [obj.Delta_, obj.G_] = lpvss2lfr_(obj.A, obj.B, C, obj.D, obj.Ts, obj.SVDTolerance);
        end
        
        function obj = set.D(obj, D)
            if isempty(D); D = zeros(obj.Ny, obj.Nu); end
            if ~all(size(D) == [obj.Ny, obj.Nu])
                error('D must be Ny-by-Nu');
            end
            [obj.Delta_, obj.G_] = lpvss2lfr_(obj.A, obj.B, obj.C, D, obj.Ts, obj.SVDTolerance);
        end
    end
end

function [A, B, C, D, Ts] = lpvlfr2ss_(syslfr)
%LPVLFR2SS Convert lpvlfr to lpvss
%
%   Syntax:
%       sysss = lpvlfr2ss_(syslfr)
%

if syslfr.Dzw ~=0 
    error('Cannot convert lpvlfr to lpvss because Dzw is not 0.');
end

Delta = syslfr.Delta;
A0 = syslfr.A0; Bu = syslfr.Bu; Bw = syslfr.Bw;
Cz = syslfr.Cz; Dzu = syslfr.Dzu;
Cy = syslfr.Cy; Dyw = syslfr.Dyw; Dyu = syslfr.Dyu;

A = A0 + Bw * Delta * Cz;
B = Bu + Bw * Delta * Dzu;
C = Cy + Dyw * Delta * Cz;
D = Dyu + Dyw * Delta * Dzu;
Ts = syslfr.Ts;

end

function [Delta, G] = lpvss2lfr_(A, B, C, D, Ts, tol)
%LPVSS2LFR_ Convert lpvss to lpvlfr
%
%   Syntax:
%       [Delta, G] = lpvss2lfr_(A, B, C, D, Ts)
%       [Delta, G] = lpvss2lfr_(A, B, C, D, Ts, tol)
%
% Inputs:
%   A, B, C, D: state-space matrices
%   Ts: sampling time
%   tol: tolerance for discarding small singular values in the creation of
%       the Delta block. Default: sqrt(eps).
%

%
%   Algorithm outline:
%
%   Matrices of lpvss depend on p as follows:
%
%       S = S0 + S1 * phi1(p) + ... + S(N-1) * phi(N-1)(p)
%
%   We want to write this expression in lpvlfr format, i.e.:
%
%       S = S0 + SL * Delta(p) * SR
%
%   To achieve this, we use SVD of S1, ..., S(N-1):
%
%       S1 * phi1(p) = SL1 * (phi1(p) * I) * SR1
%
%   By discarding singular values smaller than tol, dimensionality
%   of z & w can be reduced.
%

if nargin <= 5
    tol = sqrt(eps);
end

S = [A, B; ...
    C, D];

[n, m, N] = size(S);
S0 = zeros(n, m);

% TODO: optimize allocation of SL, SR
SL = [];
SR = [];
% Count number of non-constant scheduling dependent basis functions
% Either N or N - 1 if S is simplified
N_nonconst = sum(cellfun(@(x) ~isa(x, 'pbconst'), S.bfuncs));

% "idx" is incremented for each non-constant scheduling dependent basis
% function encountered, since constant basis functions do not contribute to
% Delta.
idx = 1;
% "s" stores the nr. of singular values associated with the matrix
% coefficient of each non-constant basis function.
s = NaN(1, N_nonconst);
% "bf" stores the non-constant basis functions of "S".
bf = cell(1, N_nonconst);

% Loop over basis functions of S.
for i=1:N
    if isa(S.bfuncs{i}, 'pbconst')
        S0 = S0 + S.matrices(:, :, i);
    else
        % Perform SVD
        [U, Sigma, V] = svd(S.matrices(:, :, i), 'econ');
        Sigma = diag(Sigma);  % Vectorizes Sigma
        % Discard small singular values
        smallSigmaIndex = find(Sigma < tol);
        U(:, smallSigmaIndex) = [];
        V(:, smallSigmaIndex) = [];
        Sigma(smallSigmaIndex) = [];
        % Diagonalize Sigma again
        Sigma = diag(Sigma);
        % Construct SL and SR
        SL = [SL, U * Sigma];   %#ok<AGROW>
        SR = [SR; V'];          %#ok<AGROW>
        % For creating Delta: store nr. of singular values
        % (for creating the matrices) and the basis function.
        s(idx) = size(Sigma, 1);
        bf{idx} = S.bfuncs{i};
        idx = idx + 1;
    end
end

% Construct Delta pmatrix object
mat = zeros(sum(s), sum(s), N_nonconst);
idx = 1;
for i=1:N_nonconst
    mat(idx:idx+s(i)-1, idx:idx+s(i)-1, i) = eye(s(i));
    idx = idx + s(i);
end
Delta = pmatrix(mat, bf, 'SchedulingTimeMap', S.timemap);

% Construct G
nx = size(A, 1);
nz = size(Delta, 1);

A = S0(1:nx, 1:nx);
Bu = S0(1:nx, nx+1:end);
Cy = S0(nx+1:end, 1:nx);
Dyu = S0(nx+1:end, nx+1:end);
if nz > 0
    Bw = SL(1:nx, :);
    Cz = SR(:, 1:nx);
    Dzw = zeros(nz);
    Dzu = SR(:, nx+1:end);
    Dyw = SL(nx+1:end, :);
else
    Bw = zeros(nx, 0);
    Cz = zeros(0, nx);
    Dzw = zeros(0, 0);
    Dzu = zeros(0, size(Bu, 2));
    Dyw = zeros(size(Cy, 1), 0);
end

G = ss(A, [Bw, Bu], [Cz; Cy], [Dzw, Dzu; Dyw, Dyu], Ts);

end

