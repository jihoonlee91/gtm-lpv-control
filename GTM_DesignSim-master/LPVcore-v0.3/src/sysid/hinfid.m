function [sys_id, gamma, info] = hinfid(G, p, sys_init, opts)
% HINFID Computes lpvlfr model given N LTI systems and scheduling values
% p.
%
%   Syntax:
%
%       [sys_id, gamma, info] = hinfid(G, p, sys_init, opts);
%       [sys_id, gamma, info] = hinfid(G, p, sys_init);
%
%   Inputs:
%       G: cell array of LTI models (tf, ss, ...).
%       p: cell array of frozen scheduling variables.
%       sys_init: initial LPV-LFR estimate.
%       opts: options for HINFSTRUCT. Default: same as HINFSTRUCT defaults.
%
%   Outputs:
%       sys_id: estimated LPV-LFR model.
%       gamma: maximum error between given LTI systems G and sys_id.
%       info: information structure returned by HINFSTRUCT.

% Validate
if nargin == 3
    opts = hinfstructOptions;
end
if length(G) ~= length(p)
    error('Number of input models must equal number of frozen scheduling variables');
end
for i=1:numel(p)
    if numel(p{i}) ~= sys_init.Np
        error(['Each supplied frozen scheduling variable must have ', int2str(sys_init.Np), ...
            ' elements.']);
    end
end

% create matrix with unknown parameters and loop over all models 
Minit = [sys_init.G.A, sys_init.G.B; sys_init.G.C, sys_init.G.D];
Mp = realp('Mp', Minit);
Mss = ss(Mp(1:sys_init.Nx, 1:sys_init.Nx), ...
    Mp(1:sys_init.Nx, sys_init.Nx+1:end), ...
    Mp(sys_init.Nx+1:end, 1:sys_init.Nx), ...
    Mp(sys_init.Nx+1:end, sys_init.Nx+1:end));
Ghat = [];
for i=1:length(G)   
    deltaP = freeze(sys_init.Delta,p{i});    
    Gp = G{i}-lft(tf(deltaP),Mss); 
    Ghat = blkdiag(Ghat,Gp);
end

% solve using hinfstruct
[Gstar, gamma, info] = hinfstruct(Ghat, opts);
Mstar = Gstar.Blocks.Mp.Value;
sys_id = lpvlfr(sys_init.Delta, ...
    ss(Mstar(1:sys_init.Nx, 1:sys_init.Nx), ...
    Mstar(1:sys_init.Nx, sys_init.Nx+1:end), ...
    Mstar(sys_init.Nx+1:end, 1:sys_init.Nx), ...
    Mstar(sys_init.Nx+1:end, sys_init.Nx+1:end)));

end