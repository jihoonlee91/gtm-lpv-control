function [data,nn,pA,pB,Ae,Be] = lpvcore2riv(lpvdata, sys)
%LPVCORE2RIV Helper function to convert LPVcore BJ IO model to RIV algorithm
%by Vincent (riv.m).
%
%   Syntax:
%       [data, nn, pA, pB, Ae, Be] = lpvcore2riv(lpvdata, sys)
%

u = lpvdata.u;
y = lpvdata.y;
p = lpvdata.p;
rho = createRho(sys.SchedulingTimeMap, p);

data = iddata(y, u, lpvdata.Ts);

% Each row in Ae, Be corresponds to a delay
% Each column in Ae, Be, corresponds to a basis functions
nf = sys.Nf + 1;
nb = sys.Nb + 1;
FMat = pmatCellCommonBfuncs(sys.F.Mat);
pA = feval(FMat{1}, rho, 'basis')';
Ae = NaN(nf, FMat{1}.numbfuncs);
for i=1:nf
    Ae(i, :) = pmatrix2rivrow_(FMat{i});
end
BMat = pmatCellCommonBfuncs(sys.B.Mat);
pB = feval(BMat{1}, rho, 'basis')';
Be = NaN(nb, BMat{1}.numbfuncs);
for i=1:nb
    Be(i, :) = pmatrix2rivrow_(BMat{i});
end

nn = [sys.Nb+1, sys.Nf, sys.InputDelay, sys.Nc, sys.Nd];

end

function coeff = pmatrix2rivrow_(A)
    mat = A.matrices;
    coeff = reshape(mat, 1, [], 1);
end

