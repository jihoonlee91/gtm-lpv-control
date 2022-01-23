function [xhtN, PtN_o, MN, P1, xhtt] = lpvssest_em_filters(sys, y, rho, u)
%LPVSSEST_EM_FILTERS An LPV-SS extended kalman filter with
%   Rauch-Tung-Striebel smoother

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


zerosnynxtmp = zeros(ny,nx);
zerosnxnxtmp = zeros(nx,nx);


%% The Robust Kalman Filter

Ptt   = zeros(nx,nx,N);
Ptt1  = zeros(nx,nx,N+1);
xhtt  = zeros(nx,N);

Ptt1(:,:,1) = sys.P1s;
xhtt1       = sys.x0;
    
    
R3 = triu(qr( [ R_s                         , zerosnynxtmp
                Ptt1(:,:,1)*CFeval(:,:,1)'  , Ptt1(:,:,1) ] ));

Ptt(:,:,1) = R3(ny+1:end,ny+1:end);
Kt = R3(1:ny,ny+1:end)' / R3(1:ny,1:ny)';

if isDnemp; xhtt(:,1) = xhtt1 + Kt*(y(:,1)-CFeval(:,:,1)*xhtt1-DFeval(:,:,1)*u(:,1));
else;       xhtt(:,1) = xhtt1 + Kt*(y(:,1)-CFeval(:,:,1)*xhtt1); end




for t = 2:N
    
    R2 = triu(qr( [ Ptt(:,:,t-1) * At(:,:,t-1)'
                    Q_s ]));

    
    Ptt1(:,:,t) = R2(1:nx,1:nx);
    
    
    R3 = triu(qr( [ R_s                         , zerosnynxtmp
                    Ptt1(:,:,t)*CFeval(:,:,t)'  , Ptt1(:,:,t)] ));
    
    Ptt(:,:,t) = R3(ny+1:end,ny+1:end);
    %         Kt         = R3(1:nx,end-ny+1:end);
    %         Kt = (Ptt1(:,:,t)'*Ptt1(:,:,t))*Ct(:,:,t)' / (Ct(:,:,t)*(Ptt1(:,:,t)'*Ptt1(:,:,t))*Ct(:,:,t)'+R);
    
    Kt = R3(1:ny,ny+1:end)' / R3(1:ny,1:ny)';
    
    xhtt1 = +At(:,:,t-1)*xhtt(:,t-1) + SdR*y(:,t-1);
    if isBtnemp; xhtt1 = xhtt1 + Bt(:,:,t-1)*u(:,t-1);    end
    
%     xhtt1     = At(:,:,t-1)*xhtt(:,t-1)+ Bt(:,:,t-1)*u(:,t-1)+ SdR*y(:,t-1);
    if isDnemp; xhtt(:,t) = xhtt1 + Kt*(y(:,t)-CFeval(:,:,t)*xhtt1-DFeval(:,:,t)*u(:,t));
    else;       xhtt(:,t) = xhtt1 + Kt*(y(:,t)-CFeval(:,:,t)*xhtt1); end
    
end


    % Faster to keep last step outside of the loop instead for if statement
R2 = triu(qr( [ Ptt(:,:,N) * At(:,:,N)'
                Q_s ]));

Ptt1(:,:,N+1) = R2(1:nx,1:nx);

xhtt1 = At(:,:,N)*xhtt(:,N)+ SdR*y(:,N);
if isBtnemp; xhtt1 = xhtt1 + Bt(:,:,N)*u(:,N);    end
       
%     xhtt1 = At(:,:,N)*xhtt(:,N)+ Bt(:,:,N)*u(:,N)+ SdR*y(:,N);  % xhtt1 for t = N+1
    

%% The Robust Kalman smoother


    % initialization needs to be done proper

    Jt = zeros(nx,nx,N);
    xhtN = zeros(nx,N+1);
    PtN = zeros(nx,nx,N+1);
    PtN_o = zeros(nx,nx,N+1);

    xhtN(:,end)    = xhtt1;
    PtN(:,:,end)   = Ptt1(:,:,end);         % t = 1,...,N+1
    PtN_o(:,:,end) = Ptt1(:,:,end)'*Ptt1(:,:,end);

    for t = N:-1:1
        
        R1p = triu(qr( [ Ptt(:,:,t) * At(:,:,t)', Ptt(:,:,t)
                       Q_s                    , zerosnxnxtmp ]));
                   
        Jt(:,:,t) = ( R1p(1:nx,1:nx) \ R1p(1:nx,nx+1:2*nx) )';        
        
%         Jt(:,:,t) = (Ptt(:,:,t)'*Ptt(:,:,t))*At(:,:,t)' / (Ptt1(:,:,t+1)'*Ptt1(:,:,t+1));
        

        xhtN(:,t) = xhtt(:,t) + Jt(:,:,t)*( xhtN(:,t+1) - At(:,:,t)*xhtt(:,t) - ...
                     SdR * y(:,t) );
                 
        if isBtnemp; xhtN(:,t) = xhtN(:,t) - Jt(:,:,t)*Bt(:,:,t)*u(:,t); end
        
%         xhtN(:,t) = xhtt(:,t) + Jt(:,:,t)*( xhtN(:,t+1) - At(:,:,t)*xhtt(:,t) - ...
%                      Bt(:,:,t)*u(:,t) - SdR * y(:,t) );

                 
        R1 = triu(qr( [ R1p
                        zerosnxnxtmp           , PtN(:,:,t+1)*Jt(:,:,t)'] ));
                 
                 
%         R1 = triu(qr( [ Ptt(:,:,t) * At(:,:,t)', Ptt(:,:,t)
%                        Q_s                    , zeros(nx,nx) 
%                        zeros(nx,nx)           , PtN(:,:,t+1)*Jt(:,:,t)'] ));

        PtN(:,:,t)   = R1(nx+1:nx*2,nx+1:nx*2);
        PtN_o(:,:,t) = PtN(:,:,t)'*PtN(:,:,t);

    end

    
%% The one-lag covariance smoother
    

    MN = zeros(nx,nx,N+1);

    MN(:,:,N)   = (eye(nx)-Kt*CFeval(:,:,N)) * At(:,:,N-1) * (Ptt(:,:,N-1)'*Ptt(:,:,N-1));
    MN(:,:,N+1) = At(:,:,N) * (Ptt(:,:,N)'*Ptt(:,:,N));
    
    
    for t = N-1:-1:2
        MN(:,:,t) = (Ptt(:,:,t)'*Ptt(:,:,t))*Jt(:,:,t-1)' + Jt(:,:,t) * ( MN(:,:,t+1)- At(:,:,t) * (Ptt(:,:,t)'*Ptt(:,:,t))) * Jt(:,:,t-1)';
    end

   P1 = PtN(:,:,1);
end

