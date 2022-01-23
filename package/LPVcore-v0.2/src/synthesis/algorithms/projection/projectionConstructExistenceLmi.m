function [LMI,gam,X,Xd,Y,Yd] = projectionConstructExistenceLmi(genplant,numAcc)
%PROJECTIONCONSTRUCTCONTROLLERLMI Constructs linear matrix inequalities for 
%projection based controller synthesis. See also <a href="matlab:help lpvl2syn">lpvl2syn</a>
%and <a href="matlab:help projectionConstructControllerLmi">projectionConstructControllerLmi</a>.
%
%   Syntax:
%       [LMI,gamma,X,Xd,Y,Yd] = projectionConstructExistenceLmi(genplant,gammafix,eta)
%
%   Inputs:
%       genplant (struct): Generalized plant structure. See 
%           <a href="matlab:help generalizedPlantToRolmip">generalizedPlantToRolmip</a>.
%       numAcc (double): Numerical accuracy parameter. Used for solving the 
%           LMIs (LMI <= numAcc).
%       
%   Outputs:
%       LMI: List of linear matrix inequalities as YALMIP constraints that
%           are used for synthesis.
%       gamma: Upper bound for the L2-gain resulting form the
%           synthesis.
%       X: X-matrix resulting from the sysnthesis as rolmipvar.
%       Xd: Derivative of the X-matrix resulting from the 
%           sysnthesis as rolmipvar. 
%       Y: Y-matrix resulting from the sysnthesis step as rolmipvar.
%       Yd: Derivative of the Y-matrix resulting from the 
%           sysnthesis as rolmipvar. 
%

%% Extract matrices from genplant
A   = genplant.A;
Bw  = genplant.Bw;
Bu  = genplant.Bu;
Cz  = genplant.Cz;
Cy  = genplant.Cy;
Dzw = genplant.Dzw;
Dzu = genplant.Dzu;
Dyw = genplant.Dyw;
st  = genplant.st;
nx  = genplant.nx;
nw  = genplant.nw;
nz  = genplant.nz;
rho = genplant.rho;
v   = genplant.v;

%% Create variables
Nx = blkdiag(null([Cy,Dyw]),eye(nz));          % size: nx+nw
Ny = blkdiag(null([Bu',Dzu']),eye(nw));        % size: nx+nz

% Check if there are rate bounds
if ~max(max(isinf(v)))
    X = rolmipvar(nx,nx,'X','sym',st,rho);
    if genplant.Ts == 0
        Xd = diff(X,'Xd',rho,v);
    else % TODO: Add support for discrete time
        warning('Discrete time synthesis does not take into account rate bounds');
        X = rolmipvar(nx,nx,'X','sym',0,0);
        Xd = zeros(nx);
    end
else
    X = rolmipvar(nx,nx,'X','sym',0,0);
    Xd = zeros(nx);
end
Y = rolmipvar(nx,nx,'Y','sym',0,0);
Yd = zeros(nx);

gam = sdpvar(1);

%% Create LMIs
LMI = [];
% Continuous Time
if genplant.Ts == 0
    H1 = Nx'*[A'*X+X*A+Xd,X*Bw,Cz';Bw'*X,-gam*eye(nw),Dzw';Cz,Dzw,-gam*eye(nz)]*Nx;     % size: nx+nw+nz
    H2 = Ny'*[A*Y+Y*A'-Yd,Y*Cz',Bw;Cz*Y,-gam*eye(nz),Dzw;Bw',Dzw',-gam*eye(nw)]*Ny;     % size: nx+nz+nw
% Discrete Time
else
    H1 = Nx'*[A'*X*A-X,A'*X*Bw',Cz';Bw'*X*A,Bw'*X*Bw-gam*eye(nw),Dzw;Cz,Dzw,-gam*eye(nz)]*Nx;    % size: nx+nw+nz
    H2 = Ny'*[A*Y*A'-Y,A*Y*Cz',Bw;Cz*Y*A',Cz*Y*Cz'-gam*eye(nz),Dzw;Bw',Dzw',-gam*eye(nw)]*Ny;    % size: nx+nz+nw
end
H3 = [X,eye(nx);eye(nx),Y];
LMI = [LMI; (H1+H1')*.5<=-numAcc*eye(size(Nx,2))];
LMI = [LMI; (H2+H2')*.5<=-numAcc*eye(size(Ny,2))];
LMI = [LMI; (H3+H3')*.5>=numAcc*eye(2*nx)];
end