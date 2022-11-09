function v = sscompare(varargin)
%SSCOMPARE Compare matrices of two or more ss objects
%
%   Syntax:
%       v = sscompare(sys1, ..., sysn)
%       v = sscompare(sys1, ..., sysn, tol)
%
%   Inputs:
%       sys1, ..., sysn: ss objects to compare
%       tol: max. absolute deviation in matrices (default: 1E-10)
%
%   Outputs:
%       v: logical indicating whether all system have numerical differences
%       with the tolerance bound.
%

narginchk(2, Inf);
if ~isa(varargin{end}, 'ss')
    tol = varargin{end};
    if tol <= 0
        error('Specified tolerance should be a positive number.');
    end
    sys = varargin(1:end-1);
else
    tol = 1E-10;
    sys = varargin;
end

for i=1:numel(sys)
    sys1 = sys{i};
    for j=1:numel(sys)
        sys2 = sys{j};
        v = sscompare_(sys1, sys2, tol);
        if ~v
            return;
        end
    end
end

end

function v = sscompare_(sys1, sys2, tol)
    M1 = [sys1.A, sys1.B; sys1.C, sys1.D];
    M2 = [sys2.A, sys2.B; sys2.C, sys2.D];
    v = max(max(abs(M1 - M2))) < tol;    
end

