function [y, t] = dtsim_MatWise(obj, p, u, p0, pf, u0, y0)
%DTSIM_ DT simulation of LPV-IO system
%
%   Syntax:
%       [y, t] = dtsim_(obj, p, u, p0, pf, u0, y0)
%
%   Inputs:
%       p: N-by-np matrix
%       u: N-by-nu matrix
%       p0: M0-by-np matrix of initial values of p
%       pf: Mf-by-np matrix of final values of p
%       u0: nb-by-nu matrix of initial values of u
%       y0: na-by-ny matrix of initial values of y
%
%   Outputs:
%       y: N-by-ny matrix of output values
%       t: column vector of N entries denoting the time of the output
%           values.

%%
na = obj.Na;
nb = obj.Nb;
ny = obj.Ny;
nu = obj.Nu;

A = obj.A;
B = obj.B;

M = size(p, 1);

p = [p0; p; pf];

% Apply InputDelay
u = [u0; u];
for i=1:nu
    u(:, i) = [zeros(obj.InputDelay(i), 1); ...
        u(1:end-obj.InputDelay(i), i)];
end


%% Create sparse A and B matrices

A_ = sparse(ny*M,ny*(M+na));
[Indi, Indj] = find(kron(speye(M),ones(ny,ny)));
for i=1:na+1
    Aeval = peval(A{i}, p);
    Atemp = sparse(Indi, Indj, Aeval(:), ny*M, ny*M);
    
    A_ = A_+ [zeros(ny*M, ny*(na-i+1)), Atemp, zeros(ny*M, ny*(i-1))];
end

B_ = sparse(ny*M,nu*(M+nb));
[Indi, Indj] = find(kron(speye(M),ones(ny,nu)));
for i=1:nb+1
    Beval = peval(B{i}, p);
    
    Btemp = sparse(Indi, Indj, Beval(:), ny*M, nu*M);

    B_ = B_ + [zeros(ny*M, nu*(nb-i+1)), Btemp, zeros(ny*M, nu*(i-1))];
end


%% Put everything into correct shape
u = u';
ny0 = na*ny;
%g: given f: to find
A_ff = A_(:,ny0+1:end);
A_fg = A_(:,1:ny0);
%B_f = B_;

%% Where the magic hapens
ya = A_ff \ (B_*u(:) - A_fg*y0(:)) ;

%% reordering output
y = reshape(ya,ny,[])';

if obj.Ts == -1
    Ts = 1;
else
    Ts = obj.Ts;
end
t=(0:Ts:(M-1)*Ts)';




end
