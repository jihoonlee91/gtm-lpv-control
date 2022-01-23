function [y, t] = ctsim_(obj, p, u, tin, solver, solverOpts)
% CTSIM_ CT simulation of LPV-IO system
%
%   Syntax:
%       [y, t] = ctsim_(obj, p, u, tin, solver, solverOpts)
%
%   Inputs:
%       p: N-by-np matrix
%       u: N-by-nu matrix
%       tin: vector of N equidistant time instants at which p and u are provided
%       solver: solver specified as function handle (e.g., @ode45)
%       solverOpts: solver options
%
%   Outputs:
%       y: N-by-ny matrix with simulated response
%       t: vector of N time instants at which y is provided
%
%   See also: LPVIO/DTSIM_

    Ts = tin(2) - tin(1);
    tm = obj.SchedulingTimeMap;
    rho = tm.eval(p,Ts);
    urho = ctevalu(obj,u,Ts);
    
    % If Na = 0, then the system represents a static gain y = B u, which
    % can be explicitly calculated
    if obj.Na == 0
        B_ = cellfun(@(x)feval(x, rho), obj.B, 'UniformOutput', false);
        B_ = cell2mat(B_);
        N = size(urho, 1);
        y = NaN(N, obj.Ny);
        for i=1:N
            y(i, :) = urho(i, :) * B_(:, :, i)';
        end
        t = tin;
        return
    end
    
    % If Na > 0, an ODE has to be solved
    x0 = zeros(obj.Na*obj.Ny, 1);
    
    fun = @(t, x) odefun(t, tin, x, rho, urho, obj);
    [t, x] = solver(fun, tin, x0, solverOpts);
    
    y = x(:,1:obj.Ny);
end

function dxdt = odefun(t, tin, x, rho, urho, obj)
    rho_ = interp1(tin, rho, t);
    u_ = interp1(tin, urho, t)';

    % compute F
    Aeval = cellfun(@(x) feval(x, rho_), obj.A, 'UniformOutput', false); 
    Fbot = -Aeval{obj.Na+1} \ cell2mat(Aeval(1:end-1));  
    Ftop = [zeros((obj.Na-1)*obj.Ny, obj.Ny), ...
        eye((obj.Na-1) * obj.Ny)];       
    F = [Ftop; Fbot];

    % compute B        
    Beval = cellfun(@(x) Aeval{obj.Na+1} \ feval(x, rho_), obj.B, ...
        'UniformOutput', false);      
    B = [ zeros((obj.Na-1) * obj.Ny, (obj.Nb + 1) * obj.Nu); ...
        cell2mat(Beval) ];

    dxdt = F * x + B * u_;

end

function urho = ctevalu(obj, u, Ts)            
    N = size(u, 1);
    urho = NaN(N, (obj.Nb+1) * obj.Nu);
    for j=1:obj.Nb+1
        diff = j - 1;
        urho(:, (j-1)*obj.Nu+1:j*obj.Nu) = u;
        for i=1:diff
            if obj.Nu > 1
                [~, urho(:, (j-1)*obj.Nu+1:j*obj.Nu)] = ...
                gradient(urho(:, (j-1)*obj.Nu+1:j*obj.Nu), Ts);
            else
                urho(:, (j-1)*obj.Nu+1:j*obj.Nu) = ...
                gradient(urho(:, (j-1)*obj.Nu+1:j*obj.Nu), Ts);
            end
        end
    end
end