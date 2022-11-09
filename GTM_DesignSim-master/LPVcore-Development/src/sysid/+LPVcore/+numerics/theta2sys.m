function sys = theta2sys(theta,sys)

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    nx = sys.nx;
    nu = sys.nu;
    ny = sys.ny;
    count = 1;

    if sys.isA == 2
        sys.A.Matrices = reshape(theta(1:nx^2*sys.npsiA),nx,nx, sys.npsiA);
        count = count+nx^2*sys.npsiA;
    elseif sys.isA == 1
        sys.A = reshape(theta(1:nx^2),nx,nx);
        count = count+nx^2;
    end

    if sys.isB == 2
        sys.B.Matrices = reshape(theta(count:count+nx*nu*sys.npsiB-1),nx,nu, sys.npsiB);
        count = count+nx*nu*sys.npsiB;
    elseif sys.isB == 1
        sys.B = reshape(theta(count:count+nx*nu-1),nx,nu);
        count = count+nx*nu;
    end

    if sys.isC == 2
        sys.C.Matrices = reshape(theta(count:count+nx*ny*sys.npsiC-1),ny,nx, sys.npsiC);
        count = count+nx*ny*sys.npsiC;
    else
        sys.C = reshape(theta(count:count+nx*ny-1),ny,nx);
        count = count+nx*ny;
    end

    if sys.isD == 2
        sys.D.Matrices = reshape(theta(count:count+ny*nu*sys.npsiD-1),ny,nu, sys.npsiD);
        count = count+ny*nu*sys.npsiD;
    elseif sys.isD == 1
        sys.D = reshape(theta(count:count+ny*nu-1),ny,nu);
        count = count+ny*nu;
    end

    if sys.isK == 2
        sys.K.Matrices = reshape(theta(count:count+ny*nx*sys.npsiK-1),nx,ny, sys.npsiK);
        count = count+ny*nx*sys.npsiK;
    elseif sys.isK == 1
        sys.K = reshape(theta(count:count+ny*nx-1),nx,ny);
        count = count+nx*ny;
    end


    if sys.isX0
        sys.x0 = theta(count:count+nx-1);
    end

end