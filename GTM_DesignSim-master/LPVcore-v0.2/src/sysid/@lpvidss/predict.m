function yp = predict(sys, data, K)
%PREDICT Summary of this function goes here
%   Detailed explanation goes here

if K ~= 1
    error("Only prediction horizon of 1 supported.");
end

if ~sys.IsInnovationForm_
    error("Only innovation form LPVIDSS models are supported.");
end

ny = sys.Ny;
np = sys.Np;
nu = sys.Nu;

% Convert matrix data to lpviddata object
doReturnNumeric = false;
if isnumeric(data)
    doReturnNumeric = true;
    assert(size(data, 2) == ny + np + nu);
    data = lpviddata(data(:, 1:ny), ...
        data(:, ny+1:ny+np), ...
        data(:, ny+np+1:ny+np+nu));
end

% Extract data
y = data.OutputData;
p = data.SchedulingData;
u = data.InputData;

A = sys.A; B = sys.B; C = sys.C; D = sys.D; K = sys.K;

%
%  Predictor form of innovation model:
%
%       xhat[k+1] = (A - K * C) xhat[k] + (B - K * D) u[k] + K y[k]
%       yhat[k] = C xhat[k] + D u[k]
%

Ap = A - K * C;
Bp = [B - K * D, K];
Cp = C;
Dp = [D, zeros(ny)];

predictor = lpvss(Ap, Bp, Cp, Dp, sys.Ts);
yp = lsim(predictor, p, [u, y], [], sys.X0);

if ~doReturnNumeric
    data.OutputData = yp;
end

end

