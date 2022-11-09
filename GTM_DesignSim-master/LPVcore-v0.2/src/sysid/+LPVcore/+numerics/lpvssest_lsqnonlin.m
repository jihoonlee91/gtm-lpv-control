function [e,J] = lpvssest_lsqnonlin(theta,u, rho, y, W, sys)

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).


    sys = LPVcore.numerics.theta2sys(theta,sys);
    
    if nargout > 1 
        [e,J] = LPVcore.numerics.lpvssest_errorWithGrad(u, rho, y, sys);
    else
        yh = LPVcore.numerics.lpvssest_predict(u, rho, y, sys);
        e = (y-yh)*W';
        e = e(:);
    end

end
