function [y, t] = dtsim_(obj, p, u, p0, pf, u0, y0)
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

na = obj.Na;
nb = obj.Nb;
ny = obj.Ny;

A = obj.A;
B = obj.B;

tm = obj.SchedulingTimeMap;

M = size(p, 1);
if ~isempty(tm.Map)
    M0 = -min(min(tm.Map), 0);
else
    M0 = 0;
end

p = [p0; p; pf];
istart = M0 + 1;
iend = istart + M - 1;

y = [y0; zeros(M, ny)];
% Apply InputDelay
for i=1:obj.Nu
    u(:, i) = [zeros(obj.InputDelay(i), 1); ...
        u(1:end-obj.InputDelay(i), i)];
end
u = [u0; u];

Aeval = cell(1, na+1);
for i=1:na+1
    Aeval{i} = peval(A{i}, p);
end

Beval = cell(1, nb+1);
for i=1:nb+1
    Beval{i} = peval(B{i}, p);
end

% loop over time
for i=istart:iend
    k = i-istart+na+1;     % index for y
    ku = i-istart+nb+1;     % index for u
    % loop over A
    for j=1:na+1
        y(k, :) = y(k, :) - ...
            (Aeval{j}(:, :, i-istart+1) * y(k-j+1, :)')';
    end
    % loop over B
    for j=1:nb+1
        y(k, :) = y(k, :) + ...
            (Beval{j}(:, :, i-istart+1) * u(ku-j+1, :)')';
    end
end

y = y(na+1:end, :);
if obj.Ts == -1
    Ts = 1;
else
    Ts = obj.Ts;
end
t=(0:Ts:(M-1)*Ts)';

end

