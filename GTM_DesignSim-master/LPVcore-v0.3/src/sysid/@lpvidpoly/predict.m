function [yp, ydtilde] = predict(sys, data, K)
%PREDICT Do K-step ahead prediction for LPVIDPOLY systems.
%
%   Syntax:
%       yp = predict(sys, data, K)
%       [yp, ydtilde] = predict(sys, data, K)
%
%   Inputs:
%       sys: lpvidpoly model
%       data: lpviddata object or numeric matrix
%       K: prediction horizon (positive integer of Inf).
%
%   Outputs:
%       yp: lpviddata object if data is an lpviddata object. The OutputData
%           property then stores the predicted output. If data is a matrix,
%           then yp is also a matrix containing the predicted output.
%       ydtilde: noise-free output: F ydtilde = B u
%

if K ~= 1
    error("Only prediction horizon of 1 supported.");
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

A = sys.A; B = sys.B; C = sys.C; D = sys.D; F = sys.F;

%
%  Starting point:
%   A y = (B/F) u + (C/D) e
% 
%  Write y as function of past values of y and e:
%   (D/C) A y = (D/C) (B/F) u + e
%   (1 + ((D/C) * A - 1)) y = yd + e
%   y = yd + (1 - (D/C) * A)y + e
%   y = yd + ys + e
%
%  Take mean to arrive at prediction:
%   yp = yd + ys
%

% yd (deterministic)
G = lpvio(F.Mat, B.Mat);
G.InputDelay = sys.InputDelay;
ydtilde = lsim(G, p, u);
if ~(iseye(C) && iseye(D))
    Hinv = lpvio(C.Mat, D.Mat);
    yd = lsim(Hinv, p, ydtilde);
else
    yd = ydtilde;
end

% ys (stochastic)
if ~(iseye(C) && iseye(D) && iseye(A))
    if iseye(D)
        Dtilde = A;
    elseif iseye(A)
        Dtilde = D;
    else
        error('Not yet supported');
        %Dtilde = D * A; % Support proper multiplication
    end
    Hinvtilde = lpvio(C.Mat, Dtilde.Mat);
    ys = y - lsim(Hinvtilde, p, y);
else
    ys = 0;
end

yp = yd + ys;

if ~doReturnNumeric
    data.OutputData = yp;
end

end

