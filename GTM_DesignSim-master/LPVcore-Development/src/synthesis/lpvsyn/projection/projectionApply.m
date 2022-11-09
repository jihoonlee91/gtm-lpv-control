function [matrices,mapping] = projectionApply(genplant,synopts)
%PROJECTIONAPPLY Applies projection method to construct continuous-time
%synthesis LMI.
%
%   Syntax:
%       [Xt,Ar,Br,Cr,Dr,N,mapping] = projectionApply(genplant,synopts)
%
%   Inputs:
%       genplant (struct): Generalized plant structure. See 
%           <a href="matlab:help lpvsynGeneralizedPlantStructure"
%           >lpvsynGeneralizedPlantStructure</a>.
%       synopts (struct): Synthesis options structure. See <a href=
%           "matlab:help lpvsynOptions">lpvsynOptions</a>.
%
%   Outputs:
%       matrices: Structure containing closed-loop matrices used for
%           projection method.
%       mapping: Stucture containing the controller (related) variables: 
%           X, Y, Ak, Bk, Ck, Dk.
%
%   See also LPVSYN.
%

%% Extract from generalized plant
A   = genplant.A;
Bw  = genplant.Bw;
Bu  = genplant.Bu;
Cz  = genplant.Cz;
Cy  = genplant.Cy;
Dzw = genplant.Dzw;
Dzu = genplant.Dzu;
Dyw = genplant.Dyw;
affine = genplant.affine;
grid = genplant.grid;
nx  = genplant.nx;
ny  = genplant.ny;
nu  = genplant.nu;
nw  = genplant.nw;
nz  = genplant.nz;
isCT = genplant.isCT;

% Other variables
paramVarLyap = synopts.parameterVaryingStorage;
perf = synopts.performance;
dependency = synopts.dependency;
feedthrough = synopts.directFeedthrough;
sysType = synopts.systemType;

%% Construct new variables
% Controller matrices
if any(dependency == 0)
    warning('Non-full dependency when using the projection might cause issues during optimization');
end
switch sysType 
    case 'affine'
        st = affine.st;
        rho = affine.rho;
        stv = affine.stv;
        vd = affine.vd;

        if dependency(1)
            mapping.Ak = rolmipvar(nx,nx,'Ak','full',st,rho);
        else
            mapping.Ak = rolmipvar(nx,nx,'Ak','full',0,0);
        end
        if dependency(2)
            mapping.Bk = rolmipvar(nx,ny,'Bk','full',st,rho);
        else
            mapping.Bk = rolmipvar(nx,ny,'Bk','full',0,0);
        end
        if dependency(3)
            mapping.Ck = rolmipvar(nu,nx,'Ck','full',st,rho);
        else
            mapping.Ck = rolmipvar(nu,nx,'Ck','full',0,0);
        end
        if feedthrough
            if dependency(4)
                mapping.Dk = rolmipvar(nu,ny,'Dk','full',st,rho);
            else
                mapping.Dk = rolmipvar(nu,ny,'Dk','full',0,0);  
            end
        else
            warning('No feedthrough when using the projection might cause issues during optimization');
            mapping.Dk = zeros(nu,ny);
        end
    case 'grid'
        dim = grid.dim; dimCell = num2cell(dim);

        if dependency(1)
            mapping.Ak = gridvar(sdpvar(nx,nx,dimCell{:},'full'));
        else
            mapping.Ak = gridvar(sdpvar(nx,nx,'full'));
        end
        if dependency(2)
            mapping.Bk = gridvar(sdpvar(nx,ny,dimCell{:},'full'));
        else
            mapping.Bk = gridvar(sdpvar(nx,ny,'full'));
        end
        if dependency(3)
            mapping.Ck = gridvar(sdpvar(nu,nx,dimCell{:},'full'));
        else
            mapping.Ck = gridvar(sdpvar(nu,nx,'full'));
        end
        if feedthrough
            if dependency(4)
                mapping.Dk = gridvar(sdpvar(nu,ny,dimCell{:},'full'));
            else
                mapping.Dk = gridvar(sdpvar(nu,ny,'full'));
            end
        else
            warning('No feedthrough when using the projection might cause issues during optimization');
            mapping.Dk = zeros(nu,ny);
        end
end

mapping.Theta = [mapping.Ak,mapping.Bk;mapping.Ck,mapping.Dk];

if isCT
    %% Continuous-time
    % Closed-loop storage function
    switch sysType
        case 'affine'
            if paramVarLyap
                mapping.X  = rolmipvar(nx,nx,'X','sym',stv,rho);  % X(p)
                mapping.Xd = diff(mapping.X,'Xd',rho,vd);         % d/dt X(p(t))
                mapping.Y  = rolmipvar(nx,nx,'Y','sym',0,0);      % Y
                mapping.Yd = zeros(nx);                           % d/dt Y = 0
            else
                mapping.X  = rolmipvar(nx,nx,'X','sym',0,0);
                mapping.Xd = zeros(nx);
                mapping.Y  = rolmipvar(nx,nx,'Y','sym',0,0);
                mapping.Yd = zeros(nx);
            end
        case 'grid'
            % For now grid-based always uses constant Lyap
            mapping.X = gridvar(sdpvar(nx));
            mapping.Xd = zeros(nx);
            mapping.Y = gridvar(sdpvar(nx));
            mapping.Yd = zeros(nx);
    end

    % Variables for projected LMI
    Xr   = [mapping.Y, eye(nx);
            eye(nx),   mapping.X];
    Ar.X = mapping.X*A+0.5*mapping.Xd;
    Ar.Y = A*mapping.Y-0.5*mapping.Yd;
    Br.X = mapping.X*Bw;
    Br.Y = Bw;
    Cr.X = Cz;
    Cr.Y = Cz*mapping.Y;
    Dr   = Dzw;
    
    % Variables for projected pole constraints
    Ar.Full = [A*mapping.Y,A;zeros(nx),mapping.X*A];

    % Projection matrices
    switch perf
        case 'h2'
            N.X1 = null([Cy,Dyw]);
            N.Y1 = blkdiag(null(Bu'),eye(nw));
            N.X2 = blkdiag(null(Cy),eye(nz));
            N.Y2 = blkdiag(eye(2*nx),null(Dzu'));
        case 'linf'
            N.X1 = null([Cy,Dyw]);
            N.Y1 = blkdiag(null(Bu'),eye(nw));
            N.X2 = blkdiag(null([Cy,Dyw]),eye(nz));
            N.Y2 = blkdiag(eye(2*nx+nz),null(Dzu'));
        case 'l2'
            N.X = blkdiag(null([Cy,Dyw]),eye(nz));
            N.Y = blkdiag(null([Bu',Dzu']),eye(nw));
        case 'passive'
            N.X = null([Cy,Dyw]);      
            N.Y = null([Bu',-Dzu']);
    end
else
    %% Discrete-time
    switch sysType
        case 'affine'
            % TODO (05/10/2021): Code below works for fixed storage function
            % but not for parameter dependent one.
            % Closed-loop storage function
            if paramVarLyap
                mapping.Xx = rolmipvar(nx,nx,'Xx','sym',stv,rho);
                mapping.XxDelta = diff(mapping.Xx,'Xxd',rho,vd);    % only valid in affine case!!!
                mapping.Xy = rolmipvar(nx,nx,'Xy','full',stv,rho);
                mapping.XyDelta = diff(mapping.Xy,'Xyd',rho,vd);
                mapping.Xz = rolmipvar(nx,nx,'Xz','sym',stv,rho);
                mapping.XzDelta = diff(mapping.Xz,'Xzd',rho,vd);
            
                mapping.J = rolmipvar(nx,nx,'J','full',st,rho);
                mapping.N = rolmipvar(nx,nx,'N','full',0,0);
                mapping.S = rolmipvar(nx,nx,'S','full',st,rho);
            else
                mapping.Xx = rolmipvar(nx,nx,'Xx','sym',0,0);
                mapping.XxDelta = zeros(nx);
                mapping.Xy = rolmipvar(nx,nx,'Xy','full',0,0);
                mapping.XyDelta = zeros(nx);
                mapping.Xz = rolmipvar(nx,nx,'Xz','sym',0,0);
                mapping.XzDelta = zeros(nx);
            
                mapping.J = rolmipvar(nx,nx,'J','full',0,0);
                mapping.N = rolmipvar(nx,nx,'N','full',0,0);
                mapping.S = rolmipvar(nx,nx,'S','full',0,0);
            end
        case 'grid'
            % For now grid-based always uses constant Lyap
            mapping.Xx = gridvar(sdpvar(nx));
            mapping.XxDelta = zeros(nx);
            mapping.Xy = gridvar(sdpvar(nx));
            mapping.XyDelta = zeros(nx);
            mapping.Xz = gridvar(sdpvar(nx));
            mapping.XzDelta = zeros(nx);
        
            mapping.J = gridvar(sdpvar(nx,nx,'full'));
            mapping.N = gridvar(sdpvar(nx,nx,'full'));
            mapping.S = gridvar(sdpvar(nx,nx,'full'));
    end
    mapping.XxPlus = mapping.Xx+mapping.XxDelta;
    mapping.XyPlus = mapping.Xy+mapping.XyDelta;
    mapping.XzPlus = mapping.Xz+mapping.XzDelta;
    
    mapping.Gt = [mapping.J, eye(nx);
                  mapping.S, mapping.N];
    mapping.Xt = [mapping.Xx,  mapping.Xy;
                  mapping.Xy', mapping.Xz];
    mapping.XtPlus = [mapping.XxPlus,  mapping.XyPlus;
                      mapping.XyPlus', mapping.XzPlus];

    % Variables for projected LMI
    Ar.U = [A*mapping.J,A];
    Ar.Y = [A;mapping.N*A];
    Br.U = Bw;
    Br.Y = [Bw;mapping.N*Bw];
    Cr.U = [Cz*mapping.J,Cz];
    Cr.Y = Cz;
    Dr   = Dzw;
    
    XrPlus.U = mapping.XxPlus;
    XrPlus.Y = mapping.XtPlus;
    Xr.U = mapping.Xt;
    Xr.Y = mapping.Xz;
    Gr.U = mapping.Gt;
    Gr.Y = mapping.N;

    % TODO (01/12/21): Variables for projected pole constraints TBD!!!
    Ar.Full = [];

    % Projection matrices
    switch perf
        case 'l2'
            N.U = blkdiag(null([Bu',Dzu']),eye(2*nx+nw));
            N.Y = blkdiag(eye(2*nx+nz),null([Cy,Dyw]));
        case 'passive'
            error('TBD');
        case 'h2'
            error('TBD');
        case 'linf'
            error('TBD');
    end

    % Extra matricses for DT
    matrices.XrPlus = XrPlus;
    matrices.Gr = Gr;
end

%% Construct matrices structure
matrices.Xr = Xr;
matrices.Ar = Ar;
matrices.Br = Br;
matrices.Cr = Cr;
matrices.Dr = Dr;
matrices.N = N;
end