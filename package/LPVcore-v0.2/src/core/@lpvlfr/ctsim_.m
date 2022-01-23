function [y, t, x] = ctsim_(obj, p, u, tin, x0, solver, solverOpts)

% get matrices of the LPV LFR model
A0 = obj.A0;
Bw = obj.Bw;
Bu = obj.Bu;
Dzw = obj.Dzw;
Cz = obj.Cz;
Cy = obj.Cy;
Dyu = obj.Dyu;
Dyw = obj.Dyw;
Dzu = obj.Dzu;
Delta = obj.Delta;
% size of Dzw*delta(P)
Nz = obj.Nz;

Ts = tin(2) - tin(1);

tm = obj.SchedulingTimeMap;

rho = tm.eval(p, Ts);

fun = @(t, x) odefun(t, x, rho, u, tin, Delta, A0, Bw, Bu, Cz, Dzu, Dzw);
[t, x] = solver(fun, tin, x0, solverOpts);

N = size(x, 1);
y = NaN(N, obj.Ny);

for i=1:N
    t_ = t(i);
    rho_ = interp1(tin, rho, t_);
    u_ = interp1(tin, u, t_);
    Deltai = feval(Delta, rho_);
    C_ = Cy + Dyw*Deltai/(eye(Nz)-Dzw*Deltai) * Cz;    
    D_ = Dyu + Dyw*Deltai/(eye(Nz)-Dzw*Deltai) * Dzu;
    y(i, :) = (C_ * x(i, :)' + D_ * u_')';
end

end


function dxdt = odefun(t, x, rho, u, tin, Delta, A0, Bw, Bu, Cz, Dzu, Dzw)

    if size(rho, 2) > 0
        rho_ = interp1(tin, rho, t);
    else
        rho_ = [];
    end
    u_ = interp1(tin, u, t);

    Nz = size(Dzw, 1);
    Delta_ = feval(Delta, rho_);
    A_ = A0 + Bw*Delta_/(eye(Nz)-Dzw*Delta_) * Cz; 
    B_ = Bu + Bw*Delta_/(eye(Nz)-Dzw*Delta_) * Dzu;
    dxdt = A_ * x + B_ * u_';

end