function [y, t, x] = dtsim_(obj, p, u, x0)

ny = obj.Ny;
nx = obj.Nx;
% get matrices of the LPV LFR model
A = obj.A0;
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

tm = obj.SchedulingTimeMap;

M = size(p, 1);
np = size(p, 2);
if ~isempty(tm.Map)
    M_min = -min(min(tm.Map), 0);
    M_max = max(max(tm.Map), 0);
else
    M_min = 0; M_max = 0;
end

p = [zeros(M_min, np); p; zeros(M_max, np)];
istart = M_min + 1;
iend = istart + M - 1;

y = NaN(M, ny);
x = NaN(M, nx);
x(1, :) = x0;

DeltaP = peval(Delta, p);
% loop over time
for i=istart:iend
    k = i-istart+1;
    DeltaPi = DeltaP(:, :, i-istart+1);
    A_ = A + Bw*DeltaPi/(eye(Nz)-Dzw*DeltaPi) * Cz; 
    B_ = Bu + Bw*DeltaPi/(eye(Nz)-Dzw*DeltaPi) * Dzu;
    C_ = Cy + Dyw*DeltaPi/(eye(Nz)-Dzw*DeltaPi) * Cz;    
    D_ = Dyu + Dyw*DeltaPi/(eye(Nz)-Dzw*DeltaPi) * Dzu;
    y(k, :) = (C_ * x(k, :)' + D_ * u(k, :)')';
    if i < iend
        x(k+1, :) = (A_ * x(k, :)' + B_ * u(k, :)')';
    end
end

if obj.Ts == -1
   obj.Ts = 1; 
end
t=(0:obj.Ts:(M-1)*obj.Ts)';

end