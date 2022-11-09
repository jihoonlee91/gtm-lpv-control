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
%      will set the property value pairs.
%   
%   See also LPVIO, LPVIDPOLY, PMATRIX, TIMEMAP.
    
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
              
            [Delta, G] = lpvss2lfr_(A, B, C, D, Ts);
            obj = obj@lpvlfr(Delta, G, varargin{:});
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
            [obj.Delta_, obj.G_] = lpvss2lfr_(A, obj.B, obj.C, obj.D, obj.Ts);
        end
        
        function obj = set.B(obj, B)
            if ~all(size(B) == [obj.Nx, obj.Nu])
                error('B must be Nx-by-Nu');
            end
            [obj.Delta_, obj.G_] = lpvss2lfr_(obj.A, B, obj.C, obj.D, obj.Ts);
        end
        
        function obj = set.C(obj, C)
            if ~all(size(C) == [obj.Ny, obj.Nx])
                error('C must be Ny-by-Nx');
            end
            [obj.Delta_, obj.G_] = lpvss2lfr_(obj.A, obj.B, C, obj.D, obj.Ts);
        end
        
        function obj = set.D(obj, D)
            if ~all(size(D) == [obj.Ny, obj.Nu])
                error('D must be Ny-by-Nu');
            end
            [obj.Delta_, obj.G_] = lpvss2lfr_(obj.A, obj.B, obj.C, D, obj.Ts);
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

function [Delta, G] = lpvss2lfr_(A, B, C, D, Ts)
%LPVSS2LFR_ Convert lpvss to lpvlfr
%
%   Syntax:
%       [Delta, G] = lpvss2lfr_(A, B, C, D, Ts)
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
%   By discarding singular values close to 0, we can reduce dimensionality
%   of z / w. However, this is not yet implemented.
%

S = [A, B; ...
    C, D];

[n, m, N] = size(S);
S0 = zeros(n, m);

% TODO: optimize allocation of SL, SR
SL = [];
SR = [];
Delta = [];
for i=1:N
    if isa(S.bfuncs{i}, 'pbconst')
        S0 = S0 + S.matrices(:, :, i);
    else
        [U, Sigma, V] = svd(S.matrices(:, :, i), 'econ');
        SL = [SL, U * Sigma];   %#ok<AGROW>
        SR = [SR; V'];          %#ok<AGROW>
        mat_ = eye(size(Sigma));
        bf_ = S.bfuncs{i};
        tm_ = S.timemap;
        Delta = blkdiag(Delta, pmatrix(mat_, {bf_}, 'SchedulingTimeMap', tm_));
    end
end

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

