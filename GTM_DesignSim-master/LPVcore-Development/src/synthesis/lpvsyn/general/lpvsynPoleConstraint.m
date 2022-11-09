function LMI = lpvsynPoleConstraint(LMI,X,XA,genplant,caseName,synopts)
%LPVSYNPOLECONSTRAINT Adds pole constraint to the LMI optimization problem
%for the synthesis algorithms.
%
%   Syntax:
%       LMI = lpvsynPoleConstraint(LMI,matrices,genplant,caseName,synopts)
%
%   Inputs:
%       LMI (lmi): YALMIP LMI array containing LMI constraints for the
%           optimization problem.
%       X (rolmipvar): Corresponding to positve definite matrix used to
%           ensure the pole constraints. For synthesis this corresponds to
%           the (transformed) closed-loop storage function matrix.
%       XA (rolmipvar): The X-matrix multiplied by the A-matrix, i.e. X*A,
%           where the A-matrix corresponds to the matrix for which the
%           pole constraints are ensured. For synthesis the A-matrix
%           corresponds to the closed-loop (transformed) state-space
%           A-matrix.
%       genplant (struct): Generalized plant structure. See 
%           <a href="matlab:help lpvsynGeneralizedPlantStructure"
%           >lpvsynGeneralizedPlantStructure</a>.
%       caseName (char): Character array or string indicating if the
%           'standard' or 'projected' constraints are to be used. The
%           projected constraints are used for projection based controller
%           synthesis.
%       synopts (structure): Stucture containing LPV synthesis options, see
%           <a href="matlab:help lpvsynOptions">lpvsynOptions</a>.
%
%   Outputs:
%       LMI: LMI array containing the new LMI constraints with the pole
%           constraints included for the optimization problem.
%
%   See also LPVSYN.
%

%% Variables
pConstr = synopts.poleConstraint;
numAcc = synopts.numericalAccuracy;
sysType = synopts.systemType;

%% Checks
Pmat = [pConstr.Q+pConstr.S,pConstr.T';pConstr.T,pConstr.U];
if genplant.Ts ~= 0 && ~isempty(Pmat)
    warning('Pole constraints are not implemented for discrete-time synthesis');
    Pmat = [];
end

%% Construct constraints
if ~isempty(Pmat)
    % Retrieve matrices
    Q = pConstr.Q;
    S = pConstr.S;
    T = pConstr.T;
    U = pConstr.U;

    switch sysType
        case 'affine'
            % Stuff to work around kron not being implemented for rolmipvars
            XAcell = homogenize(X,XA);   % Ensure same number of vertices
            Xmat = coefs(XAcell{1});     % Get 'values' at vertices
            XAmat = coefs(XAcell{2});

            if isa(Xmat,'sdpvar')   % LTI case
                dimMat = size(Xmat);
                nVertices = 1;
                Xmat = {Xmat};
                XAmat = {XAmat};
            else                    % LPV case
                dimMat = size(Xmat{1});
                nVertices = numel(Xmat);
            end
            
            % Use gridvar where each 'grid-point' is a vertex
            X = gridvar(reshape([Xmat{:}],[dimMat,nVertices]));
            XA = gridvar(reshape([XAmat{:}],[dimMat,nVertices]));
    end

    switch caseName
        % Standard constraints
        case 'standard'
            % Create constraint at each vertex of Xt and At
            
                H = [kron(X,Q)+kron(XA,S)+kron(XA',S'), kron(XA',T');
                     kron(XA,T),                        -kron(X,U)];
                nH = size(H,1);
                LMI = [LMI; (H <= -numAcc*eye(nH))];
        % Constraints for projected LMIs
        case 'projected'
            warning(['Adding pole constraints when using the projection ',...
                'might cause issues during optimization.']);

            Bu = genplant.Bu;
            Cy = genplant.Cy;
            nx = genplant.nx;
            nu = genplant.nu;
            ny = genplant.ny;

            nm = size(S,2);
            np = size(T,1);
            B1 = [zeros(nx),Bu;eye(nx),zeros(nx,nu)];
            C1 = [eye(nx),zeros(nx);zeros(ny,nx),Cy];

            Ng = null([kron(B1',S'),kron(B1',T')]);
            Nh = null([kron(C1,eye(nm)),kron(zeros(nx+ny,2*nx),zeros(nm,np))]);
            
            % Create constraint at each vertex of Xt and At
            M = [kron(X,Q)+kron(XA,S)+kron(XA',S'), kron(XA',T');
                     kron(XA,T),                        -kron(X,U)];
            H1 = Ng'*M*Ng;
            H2 = Nh'*M*Nh;
            nH1 = size(H1,1); 
            nH2 = size(H2,1);
            LMI = [LMI;
                   H1 <= -numAcc*eye(nH1);
                   H2 <= -numAcc*eye(nH2)];
    end
end
end