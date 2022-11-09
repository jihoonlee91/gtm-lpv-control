function e = lpvssest_ell2loss(u, rho, y, sys, W)

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    yh = LPVcore.numerics.lpvssest_predict(u, rho, y, sys);

        % Should be outside of the loop
    eh = (y-yh)*W';
	eh = eh(:);
    e = sqrt(eh'*eh);     % Compute the to norm

end