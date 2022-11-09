function [matrices,mapping] = transformationApply(genplant,synopts)
%TRANSFORMATIONAPPLY Applies nonlinear transformation to the
%continuous time generalized plant matrices in order to construct LMI. 
%
%   Syntax:
%       [matrices,mapping] = transformationApply(genplant,synopts)
%
%   Inputs:
%       genplant (struct): Generalized plant structure. See 
%           <a href="matlab:help lpvsynGeneralizedPlantStructure"
%           >lpvsynGeneralizedPlantStructure</a>.
%       synopts (struct): Synthesis options structure. See 
%           <a href="matlab:help lpvsynOptions">lpvsynOptions</a>.
%
%   Outputs:
%       matrices: Structure containing closed-loop transformed matrices.
%       mapping: Stucture containing the new synthesis variables.
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
st  = genplant.st;
stv = genplant.stv;     % Only used in DT
nx  = genplant.nx;
ny  = genplant.ny;
nu  = genplant.nu;
rho = genplant.rho;
vd  = genplant.vd;      % Only used in DT

% Other variables
paramVarLyap = synopts.parameterVaryingStorage;
dependency = synopts.dependency;
feedthrough = synopts.directFeedthrough;
isCT = genplant.isCT;

%% Construct new variables
if isCT
    %% Continuous-time
    % Create transformed closed-loop storage function matrices
    if paramVarLyap
        warning(strcat("Parameter-varying storage not supported for ",...
                 "transformation method. Set method option to 'projection', ",...
                 "see help lpvsynOptions."));
             
        mapping.X = rolmipvar(nx,nx,'X','sym',0,0);
        mapping.Xd = zeros(nx,nx);
        mapping.Y = rolmipvar(nx,nx,'Y','sym',0,0);
        mapping.Yd = zeros(nx,nx);
    else
        mapping.X = rolmipvar(nx,nx,'X','sym',0,0);
        mapping.Xd = zeros(nx,nx);
        mapping.Y = rolmipvar(nx,nx,'Y','sym',0,0);
        mapping.Yd = zeros(nx,nx);
    end
    
    % Create 'transformed controller' variables
    if dependency(1)
        mapping.K = rolmipvar(nx,nx,'K','full',st,rho);
    else
        mapping.K = rolmipvar(nx,nx,'K','full',0,0);
    end
    if dependency(2)
        mapping.L = rolmipvar(nx,ny,'L','full',st,rho);
    else
        mapping.L = rolmipvar(nx,ny,'L','full',0,0);
    end
    if dependency(3)
        mapping.M = rolmipvar(nu,nx,'M','full',st,rho);
    else
        mapping.M = rolmipvar(nu,nx,'M','full',0,0);
    end
    if feedthrough
        if dependency(4)
            mapping.N = rolmipvar(nu,ny,'N','full',st,rho);
        else
            mapping.N = rolmipvar(nu,ny,'N','full',0,0);  
        end
    else
        mapping.N = zeros(nu,ny);
    end
    
    % Create transformed closed-loop marices
    matrices.Xt = [mapping.Y, eye(nx);
                   eye(nx), mapping.X];
    matrices.At = [A*mapping.Y+Bu*mapping.M-0.5*mapping.Yd,  A+Bu*mapping.N*Cy;
                   mapping.K,                                mapping.X*A+mapping.L*Cy+0.5*mapping.Xd];
    matrices.Bt = [Bw+Bu*mapping.N*Dyw;
                   mapping.X*Bw+mapping.L*Dyw];
    matrices.Ct = [Cz*mapping.Y+Dzu*mapping.M, Cz+Dzu*mapping.N*Cy];
    matrices.Dt = Dzw+Dzu*mapping.N*Dyw;
else
    %% Discrete-time
    % Create transforemd closed-loop storage function and slack variable matrices
    if paramVarLyap
        mapping.Xx = rolmipvar(nx,nx,'Xx','sym',stv,rho);
        mapping.XxDelta = diff(mapping.Xx,'Xxd',rho,vd);    % only valid in affine case!!!
        mapping.Xy = rolmipvar(nx,nx,'Xy','full',stv,rho);
        mapping.XyDelta = diff(mapping.Xy,'Xyd',rho,vd);
        mapping.Xz = rolmipvar(nx,nx,'Xz','sym',stv,rho);
        mapping.XzDelta = diff(mapping.Xz,'Xzd',rho,vd);
    else
        mapping.Xx = rolmipvar(nx,nx,'Xx','sym',0,0);
        mapping.XxDelta = zeros(nx);
        mapping.Xy = rolmipvar(nx,nx,'Xy','full',0,0);
        mapping.XyDelta = zeros(nx);
        mapping.Xz = rolmipvar(nx,nx,'Xz','sym',0,0);
        mapping.XzDelta = zeros(nx);
    end
    mapping.XxPlus = mapping.Xx+mapping.XxDelta;
    mapping.XyPlus = mapping.Xy+mapping.XyDelta;
    mapping.XzPlus = mapping.Xz+mapping.XzDelta;
    
    mapping.J = rolmipvar(nx,nx,'J','full',0,0);
    mapping.N = rolmipvar(nx,nx,'N','full',0,0);
    mapping.S = rolmipvar(nx,nx,'S','full',0,0);

    % Create 'transformed controller' variables
    if dependency(1)
        mapping.U = rolmipvar(nx,nx,'U','full',st,rho);
    else
        mapping.U = rolmipvar(nx,nx,'U','full',0,0);
    end
    if dependency(2)
        mapping.V = rolmipvar(nx,ny,'V','full',st,rho);
    else
        mapping.V = rolmipvar(nx,ny,'V','full',0,0);
    end
    if dependency(3)
        mapping.W = rolmipvar(nu,nx,'W','full',st,rho);
    else
        mapping.W = rolmipvar(nu,nx,'w','full',0,0);
    end
    if feedthrough
        if dependency(4)
            mapping.X = rolmipvar(nu,ny,'X','full',st,rho);
        else
            mapping.X = rolmipvar(nu,ny,'X','full',0,0);  
        end
    else
        mapping.X = zeros(nu,ny);
    end
    
    % Create transformed closed-loop marices
    matrices. Gt = [mapping.J, eye(nx);
                    mapping.S, mapping.N];
    matrices.Xt = [mapping.Xx,  mapping.Xy;
                   mapping.Xy', mapping.Xz];
    matrices.XtPlus = [mapping.XxPlus,  mapping.XyPlus;
                       mapping.XyPlus', mapping.XzPlus];
    
    matrices.At = [A*mapping.J+Bu*mapping.W, A+Bu*mapping.X*Cy;
                   mapping.U,                mapping.N*A+mapping.V*Cy];
    matrices.Bt = [Bw+Bu*mapping.X*Dyw;
                   mapping.N*Bw+mapping.V*Dyw];
    matrices.Ct = [Cz*mapping.J+Dzu*mapping.W,Cz+Dzu*mapping.X*Cy];
    matrices.Dt = Dzw+Dzu*mapping.X*Dyw;
end
end