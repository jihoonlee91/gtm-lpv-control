function res = simulLPV(u, F,fncA ,B, fncB)

%function which simulate a LPV model defined by
%   {F(q,scheA)y(tk) = [B(q,scheB)] u(tk) 
%==========================================================
%CALL
%     y = simulLPV(u, F,scheA ,B, scheB)
%==========================================================
% INPUTS
%   -u: Input signal   size (1,N)
%   -F: LPV Denominator size(n_a,n_alpha)
%   -scheA: Scheduling parameters size(n_alpha, N)
%   -B: LPV Numerator size(n_b,n_beta)
%   -scheB: Scheduling parameters size(n_beta, N);
%==========================================================
% OUTPUT
%  -y: simulated output size (1,N)
%==========================================================
if (size(F,2) ~= size(fncA,1))|| (size(B,2) ~= size(fncB,1))
    error('size of polynoms and scheduling parameters do not correspond (see help)')
end

na  = size(F,1);
nb  = size(B,1);
nk=0;
it = 1;
sumB= sum(abs(B),2);
while sumB(it)==0
    nk=nk+1;
    it =it+1;
end


nmax = max([na+1 nb+nk]);



yam2(1:nmax) = 0;
udummy(1:nmax) = 0;
udummy = [udummy u'];

Ae = F*fncA;
Be = B*fncB;


%     Be = [zeros(nk,1) Be];
for ii=nmax+1:length(u)+nmax
    realindex = ii-nmax;
    yam2(ii)=0;

    for iF=2:na
        yam2(ii)= yam2(ii) - yam2(ii-iF+1).*Ae(iF,realindex);
    end
 
    for iB=it:nb
        yam2(ii) = yam2(ii) +  udummy(ii-iB+1)*Be(iB,realindex);
    end
    
end

res=yam2(nmax+1:end)';



