% this function linearize the system

function [Alon,Blon,Alat,Blat] = lin_mod(x_trim_,u_trim_,eta0)

dx0 = state_eq_lin(x_trim_,u_trim_,eta0);

ptrb = 1e-9;

N = 12;
M = 4;

dfdx = zeros(N,N); 
for n = 1:N
    ptrbvecx = zeros(N,1);
    ptrbvecx(n) = ptrb; 
    dx = state_eq_lin(x_trim_+ptrbvecx,u_trim_,eta0);
    dfdx(:,n) = (dx-dx0)/ptrb;
end

dfdu = zeros(N,M); 
for m = 1:M
    ptrbvecu = zeros(M,1);
    ptrbvecu(m) = ptrb; 
    dx = state_eq_lin(x_trim_,u_trim_+ptrbvecu,eta0);
    dfdu(:,m) = (dx-dx0)/ptrb; 
end

Elon1 = zeros(4,N);
Elon1(1,1) = 1; % VT
Elon1(2,2) = 1; % alp
Elon1(3,5) = 1; % q
Elon1(4,8) = 1; % th

Elon2 = zeros(2,M);
Elon2(1,1) = 1; % delt
Elon2(2,2) = 1; % dele

Elat1 = zeros(4,N);
Elat1(1,3) = 1; % beta
Elat1(2,4) = 1; % p
Elat1(3,6) = 1; % r
Elat1(4,7) = 1; % ph

Elat2 = zeros(2,M);
Elat2(1,3) = 1; % dela
Elat2(2,4) = 1; % delr

% Beard and McLain, "Small Unmanned Aircraft Theory and Practice", pp. 284
Alon = Elon1*dfdx*Elon1';
Blon = Elon1*dfdu*Elon2';

Alat = Elat1*dfdx*Elat1';
Blat = Elat1*dfdu*Elat2';

end