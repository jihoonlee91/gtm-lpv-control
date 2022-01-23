function yh = lpvssest_predict(u, rho, y, sys)
%LPVSSEST_PREDICT Computes the predicted output of an innovation model

% We assume that either (A, B, K) is nonempty (existance of state dynamics)
% and C nonempty (state is observed). Else we do not require a state. This
% implies that AT = A-K*C always exists and C always exists

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

nx = sys.nx;
ny = sys.ny;
nu = sys.nu;

u = u';
y = y';

N = size(u,2);

xh  = zeros(nx,N+1);
if sys.isX0; xh(:,1) = sys.x0; end  % Set initial condition
yh  = zeros(ny,N);


% precompute the matrices over the scheduling trajectory
AFeval = feval(sys.A,rho); 
BFeval = feval(sys.B,rho); 
CFeval = feval(sys.C,rho); 
DFeval = feval(sys.D,rho); 
KFeval= feval(sys.K,rho); 

isDnemp = ~isempty(sys.D);
isKnemp = ~isempty(sys.K);

sys.isA = ~isempty(sys.A);
sys.isB = ~isempty(sys.B);
sys.isC = ~isempty(sys.C);
sys.isD = ~isempty(sys.D);
sys.isK = ~isempty(sys.K);

if sys.isA ~= 0 && sys.isC ~= 0 && sys.isK ~= 0
    ATFeval = zeros(nx, nx, N);
    for i = 1:N; ATFeval(:,:,i) = AFeval(:,:,i) - KFeval(:,:,i)*CFeval(:,:,i); end
    
elseif sys.isC ~= 0 && sys.isK ~= 0
    ATFeval = zeros(nx, nx, N);
    for i = 1:N; ATFeval(:,:,i) = - KFeval(:,:,i)*CFeval(:,:,i); end
    
else  % sys.isA ~= 0
    ATFeval = AFeval;
end

if sys.isB ~= 0 && sys.isD ~= 0 && sys.isK ~= 0
    isBTnemp = true;
    BTFeval = zeros(nx, nu, N);
    for i = 1:N; BTFeval(:,:,i) = BFeval(:,:,i) - KFeval(:,:,i)*DFeval(:,:,i); end
elseif sys.isD ~= 0 && sys.isK ~= 0
    isBTnemp = true;
    BTFeval = zeros(nx, nu, N);
    for i = 1:N; BTFeval(:,:,i) = - KFeval(:,:,i)*DFeval(:,:,i); end
elseif sys.isB ~= 0
    isBTnemp = true;
    BTFeval = BFeval;
else
    isBTnemp = false;
end


for t = 1:N

    xh(:,t+1) = ATFeval(:,:,t)*xh(:,t);
    if isBTnemp; xh(:,t+1) = xh(:,t+1)+  BTFeval(:,:,t)*u(:,t); end
    if isKnemp;  xh(:,t+1) = xh(:,t+1)+  KFeval(:,:,t)*y(:,t);  end
    
    yh(:,t)   = CFeval(:,:,t)*xh(:,t);
    if isDnemp;  yh(:,t)   = yh(:,t) + DFeval(:,:,t)*u(:,t);    end
         
end

    yh = yh';

end