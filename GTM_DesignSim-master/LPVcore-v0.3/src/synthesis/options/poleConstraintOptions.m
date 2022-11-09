function P = poleConstraintOptions(varargin)
%poleConstraintOptions Constructs matrices for closed-loop pole constraints
%use for synthesis. Multiple name-value pairs can be given for multiple
%constraints. 
%
%   Syntax:
%       out = poleConstraintOptions(...)
%   
%   Optional name-value pairs:
%       'rightHalfPlane': Constraint eigenvalues to Re(eig) > value, i.e.
%           a right half-plane.
%       'leftHalfPlane': Constraint eigenvalues to Re(eig) < value, i.e.
%           a left half-plane.
%       'circle': Constraint eigenvalues to |eig| < value, i.e. an open
%           circle with a radius of 'value'.
%       'sector': Constraint eigenvalues to Re(eig)tan(value) < -|Im(eig)|,
%           i.e. a sector in the left half complex plane with angle of
%           'value' [rad].
%       'custom': Custom pole constraint given as a structure with fields,
%           Q, S, T, and U, corresponding to the constraint:
%           [ I ]' [ Q  S ] [ I ]
%           [ z ]  [ S' R ] [ z ] < 0,
%           for all closed-loop eigenvalues 'z', where R >= 0 s.t. it is 
%           factorizable as R = T' U^(-1) T, with U > 0.
%
%   Outputs:
%       out: Structure containing Q, S, T and U, matrices corresponding to
%           the pole constraint.
%
%   See also LPVSYNOPTIONS, LPVSYN.
%

%% Parse inputs
opt = inputParser;

addParameter(opt,'rightHalfPlane',[],@valueCheck);
addParameter(opt,'leftHalfPlane',[],@valueCheck);
addParameter(opt,'sector',[],@valueCheck);
addParameter(opt,'circle',[],@valueCheck);
addParameter(opt,'custom',[],@structCheck)

parse(opt,varargin{:});

valueRHP    = opt.Results.rightHalfPlane;
valueLHP    = opt.Results.leftHalfPlane;
valueSector = opt.Results.sector;
valueCircle = opt.Results.circle;
valueCustom = opt.Results.custom;

% If nothing is set
if all(cellfun(@isempty,struct2cell(opt.Results)))
    P = [];
else
    %% Construct pole constraint matrix
    % Constraints are defined as
    % [ I ]' [ Q  S ] [ I ]
    % [ z ]  [ S' R ] [ z ] < 0
    % where z is a complex number.
    % Furthermore, R >= 0 s.t. it is factorizable as R = T' U^(-1) T, with 
    % U > 0.
    
    P.Q = [];
    P.S = [];
    P.T = [];
    P.U = [];
    
    % right half plane
    if ~isempty(valueRHP)
        P.Q = blkdiag(P.Q,2*valueRHP);
        P.S = blkdiag(P.S,-1);
        P.T = blkdiag(P.T,0);
        P.U = blkdiag(P.U,1);
    end
    % left half plane
    if ~isempty(valueLHP)
        P.Q = blkdiag(P.Q,-2*valueLHP);
        P.S = blkdiag(P.S,1);
        P.T = blkdiag(P.T,0);
        P.U = blkdiag(P.U,1);
    end
    % sector
    if ~isempty(valueSector)
        P.Q = blkdiag(P.Q,zeros(2));
        P.S = blkdiag(P.S,[sin(valueSector),cos(valueSector);...
            -cos(valueSector),sin(valueSector)]);
        P.T = blkdiag(P.T,zeros(1,2));
        P.U = blkdiag(P.U,1);
    end
    % circle
    if ~isempty(valueCircle)
        P.Q = blkdiag(P.Q,-valueCircle^2);
        P.S = blkdiag(P.S,0);
        P.T = blkdiag(P.T,1);
        P.U = blkdiag(P.U,1);
    end
    % custom
    if ~isempty(valueCustom)
        P.Q = blkdiag(P.Q,valueCustom.Q);
        P.S = blkdiag(P.S,valueCustom.S);
        P.T = blkdiag(P.T,valueCustom.T);
        P.U = blkdiag(P.U,valueCustom.U);
    end
    
    % remove unnecessary rows from L (and corresponding rows+columns from I)
    zeroRowL = all(P.T == 0,2);
    P.T(zeroRowL,:)= [];
    P.U = P.U(~zeroRowL,~zeroRowL);
    
    % if all rows are removed, R = 0 --> R = [] (to prevent unnecessary
    % constraint)
    if isempty(P.U)
        P.U = [];
        P.T = [];
    end
end
end

%% Local functions
% Check value
function out = valueCheck(x)
    out = (numel(x) == 1) && isnumeric(x);
end

% Check custom value
function out = structCheck(x)
    qCheck = isnumeric(x.Q) && issymmetric(x.Q);
    sCheck = isnumeric(x.S) && (size(x.S,1) == size(x.Q,1)) && (size(x.S,2) == size(x.T,2));
    lCheck = isnumeric(x.T) && (size(x.T,1) == size(x.U,1));
    iCheck = isnumeric(x.U) && (eye(size(x.U,1)) == x.I);
    out = qCheck && sCheck && lCheck && iCheck;
end