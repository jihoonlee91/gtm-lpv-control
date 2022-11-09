function [matrices,mapping] = transformationApply(genplant,synopts)
%TRANSFORMATIONAPPLY Applies nonlinear transformation to the generalized
%plant matrices in order to construct LMI. 
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
nx  = genplant.nx;
ny  = genplant.ny;
nu  = genplant.nu;
affine = genplant.affine;
grid = genplant.grid;

% Other variables
paramVarLyap = synopts.parameterVaryingStorage;
dependency = synopts.dependency;
feedthrough = synopts.directFeedthrough;
sysType = synopts.systemType;
isCT = genplant.isCT;

%% Construct new variables
if isCT
    %% Continuous-time
    % Create transformed closed-loop storage function matrices
    switch sysType
        case 'affine'
            st = affine.st;
            rho = affine.rho;

            % For now transformation method always uses constant Lyap
            mapping.X = rolmipvar(nx,nx,'X','sym',0,0);
            mapping.Xd = zeros(nx);
            mapping.Y = rolmipvar(nx,nx,'Y','sym',0,0);
            mapping.Yd = zeros(nx);
            
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

        case 'grid'
            dim = grid.dim; dimCell = num2cell(dim);

            mapping.X = gridvar(sdpvar(nx));
            mapping.Xd = zeros(nx);
            mapping.Y = gridvar(sdpvar(nx));
            mapping.Yd = zeros(nx);

            % Create 'transformed controller' variables
            if dependency(1)
                mapping.K = gridvar(sdpvar(nx,nx,dimCell{:},'full'));
            else
                mapping.K = gridvar(sdpvar(nx,nx,'full'));
            end
            if dependency(2)
                mapping.L = gridvar(sdpvar(nx,ny,dimCell{:},'full'));
            else
                mapping.L = gridvar(sdpvar(nx,ny,'full'));
            end
            if dependency(3)
                mapping.M = gridvar(sdpvar(nu,nx,dimCell{:},'full'));
            else
                mapping.M = gridvar(sdpvar(nu,nx,'full'));
            end
            if feedthrough
                if dependency(4)
                    mapping.N = gridvar(sdpvar(nu,ny,dimCell{:},'full'));
                else
                    mapping.N = gridvar(sdpvar(nu,ny,'full'));
                end
            else
                mapping.N = zeros(nu,ny);
            end
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
    switch sysType
        case 'affine'
            % Create transforemd closed-loop storage function and slack variable matrices
            st = affine.st;
            stv = affine.stv;
            rho = affine.rho;
            vd = affine.vd;
        
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

        case 'grid'
            % Create transforemd closed-loop storage function and slack variable matrices
            dim = grid.dim; dimCell = num2cell(dim);
        
            % For now transformation method always uses constant Lyap
            mapping.Xx = gridvar(sdpvar(nx));
            mapping.XxDelta = zeros(nx);
            mapping.Xy = gridvar(sdpvar(nx));
            mapping.XyDelta = zeros(nx);
            mapping.Xz = gridvar(sdpvar(nx));
            mapping.XzDelta = zeros(nx);

            mapping.XxPlus = mapping.Xx+mapping.XxDelta;
            mapping.XyPlus = mapping.Xy+mapping.XyDelta;
            mapping.XzPlus = mapping.Xz+mapping.XzDelta;
            
            mapping.J = gridvar(sdpvar(nx,nx,'full'));
            mapping.N = gridvar(sdpvar(nx,nx,'full'));
            mapping.S = gridvar(sdpvar(nx,nx,'full'));
        
            % Create 'transformed controller' variables
            if dependency(1)
                mapping.U = gridvar(sdpvar(nx,nx,dimCell{:},'full'));
            else
                mapping.U = gridvar(sdpvar(nx,nx,'full'));
            end
            if dependency(2)
                mapping.V = gridvar(sdpvar(nx,ny,dimCell{:},'full'));
            else
                mapping.V = gridvar(sdpvar(nx,ny,'full'));
            end
            if dependency(3)
                mapping.W = gridvar(sdpvar(nu,nx,dimCell{:},'full'));
            else
                mapping.W = gridvar(sdpvar(nu,nx,'full'));
            end
            if feedthrough
                if dependency(4)
                    mapping.X = gridvar(sdpvar(nu,ny,dimCell{:},'full'));
                else
                    mapping.X = gridvar(sdpvar(nu,ny,'full')); 
                end
            else
                mapping.X = zeros(nu,ny);
            end
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