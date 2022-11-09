function [yh, xhtt] = lpvssest_em_predict(sys, y, rho, u)
%LPVSSEST_EM_PREDICT Computation of the LPV-SS extended kalman filter
%

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).


nx = sys.nx;
nu = sys.nu;
ny = sys.ny;
N  = size(y,1);

Q = sys.NoiseVariance(1:nx,1:nx);
S = sys.NoiseVariance(1:nx,nx+1:end);
R = sys.NoiseVariance(nx+1:end,nx+1:end);

SdR = S / R;

y = y';
u = u';


Qb = (Q-SdR*S');
Q_s     = idpack.cholcov(Qb,'upper');
R_s    = idpack.cholcov(R,'upper');


% precompute the matrices over the scheduling trajectory
if     sys.isA == 2; AFeval = feval(sys.A,rho);
elseif sys.isA == 1; AFeval = repmat(sys.A,[1 1 N]); end

if     sys.isB == 2; BFeval = feval(sys.B,rho);
elseif sys.isB == 1; BFeval = repmat(sys.B,[1 1 N]); end

if     sys.isC == 2; CFeval = feval(sys.C,rho);
elseif sys.isC == 1; CFeval = repmat(sys.C,[1 1 N]); end

if     sys.isD == 2; DFeval = feval(sys.D,rho);
elseif sys.isD == 1; DFeval = repmat(sys.D,[1 1 N]); end


if sys.isD == 0; isDnemp = false;
else;            isDnemp = true; end

if sys.isA ~= 0
    At = zeros(nx, nx, N);
    for i = 1:N; At(:,:,i) = AFeval(:,:,i) - SdR*CFeval(:,:,i); end
else
    At = zeros(nx, nx, N);
    for i = 1:N; At(:,:,i) = - SdR*CFeval(:,:,i); end
end

if sys.isB ~= 0 && sys.isD ~= 0
    isBtnemp = true;
    Bt = zeros(nx, nu, N);
    for i = 1:N; Bt(:,:,i) = BFeval(:,:,i) - SdR*DFeval(:,:,i); end
elseif sys.isB ~= 0
    isBtnemp = true;
    Bt = BFeval;
else
    isBtnemp = false;
end
    
    
%% The Robust Kalman Filter

% Ptt   = zeros(nx,nx,N);
Ptt1  = zeros(nx,nx,N+1);
xhtt  = zeros(nx,N);
yh     = zeros(ny,N);

Ptt1(:,:,1) = sys.P1s;
xhtt1       = sys.x0;
    
    
R3 = triu(qr( [ R_s                         , zeros(ny,nx)
                Ptt1(:,:,1)*CFeval(:,:,1)'  , Ptt1(:,:,1) ] ));

Ptt = R3(ny+1:end,ny+1:end);
Kt = R3(1:ny,ny+1:end)' / R3(1:ny,1:ny)';

if isDnemp
    xhtt(:,1) = xhtt1 + Kt*(y(:,1)-CFeval(:,:,1)*xhtt1-DFeval(:,:,1)*u(:,1));
    yh(:,1) = CFeval(:,:,1)*xhtt(:,1)+DFeval(:,:,1)*u(:,1);
else
    xhtt(:,1) = xhtt1 + Kt*(y(:,1)-CFeval(:,:,1)*xhtt1);
    yh(:,1) = CFeval(:,:,1)*xhtt(:,1);
end




for t = 2:N
    
    R2 = triu(qr( [ Ptt * At(:,:,t-1)'
                    Q_s ]));

    
    Ptt1(:,:,t) = R2(1:nx,1:nx);
    
    
    R3 = triu(qr( [ R_s                         , zeros(ny,nx)
                    Ptt1(:,:,t)*CFeval(:,:,t)'  , Ptt1(:,:,t)] ));
    
    Ptt = R3(ny+1:end,ny+1:end);

    
    Kt = (R3(1:ny,1:ny) \ R3(1:ny,ny+1:end))';
    
    xhtt1 = +At(:,:,t-1)*xhtt(:,t-1) + SdR*y(:,t-1);
    if isBtnemp; xhtt1 = xhtt1 + Bt(:,:,t-1)*u(:,t-1);    end
    

    if isDnemp
        xhtt(:,t) = xhtt1 + Kt*(y(:,t)-CFeval(:,:,t)*xhtt1-DFeval(:,:,t)*u(:,t));
        yh(:,t) = CFeval(:,:,t)*xhtt(:,t)+DFeval(:,:,t)*u(:,t);
    else
        xhtt(:,t) = xhtt1 + Kt*(y(:,t)-CFeval(:,:,t)*xhtt1);
        yh(:,t) = CFeval(:,:,t)*xhtt(:,t);
    end
    
end

xhtt = xhtt';
yh = yh';

end

