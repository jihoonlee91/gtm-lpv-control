function sys = randlpvidpoly(sys, keepZeroes)
%RANDLPVIDPOLY Generates an LPVIDPOLY object with random coefficients
%
%   Syntax:
%       sys = randlpvidpoly(sys)
%       sys = randlpvidpoly(sys, keepZeroes)
%
%   Inputs:
%       sys: lpvidpoly object
%       keepZeroes: (logical) whether 0s are kept as 0s. Default: true
%
%   Outputs:
%       randsys: lpvidpoly object with the same IO size, basis functions
%       and orders as sys, but random coefficients.

if nargin == 1
    keepZeroes = true;
end

for i=2:sys.Na+1
    if keepZeroes && sys.A.Mat{i} ~= 0
        sys.A.Mat{i} = randpmatrix(sys.A.Mat{i});
    end
end
for i=1:sys.Nb
    if keepZeroes && sys.B.Mat{i} ~= 0
        sys.B.Mat{i} = randpmatrix(sys.B.Mat{i});
    end
end
for i=2:sys.Nc+1
    if keepZeroes && sys.C.Mat{i} ~= 0
        sys.C.Mat{i} = randpmatrix(sys.C.Mat{i});
    end
end
for i=2:sys.Nd+1
    if keepZeroes && sys.D.Mat{i} ~= 0
        sys.D.Mat{i} = randpmatrix(sys.D.Mat{i});
    end
end
for i=2:sys.Nf+1
    if keepZeroes && sys.F.Mat{i} ~= 0
        sys.F.Mat{i} = randpmatrix(sys.F.Mat{i});
    end
end

end

