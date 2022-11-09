function [V,J] = lpvssest_ell2lossWithGrad(theta, u, rho, y, W, sys)

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    sys = LPVcore.numerics.theta2sys(theta,sys);
    
    [yh,J] = LPVcore.numerics.lpvssest_errorWithGrad(u, rho, y, sys);
    
    J = J';
    
        % Should be outside of the loop
    eh = (y-yh)*W';
	eh = eh(:);
    V = sqrt(eh'*eh);     % Compute the to norm

end
