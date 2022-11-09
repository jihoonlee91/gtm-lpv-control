function K = projectionConstructController(genplant,Theta)
%PROJECTIONCONSTRUCTCONTROLLER Constructs the controller resulting from 
%projection based controller synthesis. See also <a href="matlab:help lpvl2syn">lpvl2syn</a>.
%
%   Syntax:
%       [K,theta] = projectionConstructController(genplant,Theta)
%
%   Inputs:
%       genplant (struct): Generalized plant structure. See 
%           <a href="matlab:help generalizedPlantToRolmip">generalizedPlantToRolmip</a>.
%       Theta (rolmipvar): Variable containing the controller matrices in
%           the form: Theta = [ Ak, Bk; Ck, Dk ].
%       
%   Outputs:
%       K: Constructed controller as lpvss object.
%       theta: Cell array containing the affine controller matrices where:
%           theta = { Ak, Bk; Ck, Dk }
%           where Ak is the affine parameter dependent matrix of the form
%           Ak = Ak0 + Ak1 * rho_1 + Ak2 *rho_2 + ...
%           where Ak{1} = Ak0, Ak{2} = Ak1, Ak{3} = Ak2, ... .
%

%% Extract from genplant
st = genplant.st;
nx = genplant.nx;
ny = genplant.ny;
nu = genplant.nu;
np = genplant.np;
p  = genplant.p;

%% Extract Controller
Theta = value(Theta);
Theta_ = cell(1,np+1);

Ak = cell(1,np+1);
Bk = cell(1,np+1);
Ck = cell(1,np+1);
Dk = cell(1,np+1);

isLPV = ~isa(Theta,'double');

% LPV case
if isLPV
    Theta_{1} = evalpar(Theta,st{1});
    for i = 1:np+1
        if i > 1
            Theta_{i} = evalpar(Theta,st{i})-Theta_{1};
        end
        Ak{i} = Theta_{i}(1:nx,1:nx);
        Bk{i} = Theta_{i}(1:nx,end-ny+1:end);
        Ck{i} = Theta_{i}(end-nu+1:end,1:nx);
        Dk{i} = Theta_{i}(end-nu+1:end,end-ny+1:end);
    end
    % LTI case
else
    Ak{1} = Theta(1:nx,1:nx);
    Bk{1} = Theta(1:nx,end-ny+1:end);
    Ck{1} = Theta(end-nu+1:end,1:nx);
    Dk{1} = Theta(end-nu+1:end,end-ny+1:end);
end

ak = Ak{1};
bk = Bk{1};
ck = Ck{1};
dk = Dk{1};

if isLPV
    for i = 2:np+1
        ak = ak+Ak{i}*p{i-1};
        bk = bk+Bk{i}*p{i-1};
        ck = ck+Ck{i}*p{i-1};
        dk = dk+Dk{i}*p{i-1};
    end
end

K = lpvss(ak,bk,ck,dk);
end