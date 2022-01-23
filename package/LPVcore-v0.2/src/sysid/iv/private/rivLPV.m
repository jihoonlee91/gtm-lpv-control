function [thsriv, Ariv, Briv, Ce,De,infos,ysim]=rivLPV(data,nn,tol,maxiter,pA,pB,Ae,Be)
% LPV-RIV  Computes Refined IV-estimates for LPV-BJ models of type
%    {F(q,scheA)xi(tk) = [B(q,scheB)] u(tk-nk)
%    {y(tk) = xi(tk)+ [C(q)/D(q)]e(t)
% 
%   ======================================================================
%  CALL:
%   [theta,C,D,infos,ysim] = 
%      rivLPV(DATA,NN,TOL,MAXITER,Ascheduling,Bscheduling,[Ainit],[Binit])
%
%   ======================================================================
%    OUTPUTS:
%    -theta : returned as the estimated parameter vector of the BJ model
%    - C : noise model numerator
%    - D : noise model denominator
%
%   - infos : information structure containing:
%        .J	: estimated noise variance 
%        .converge : 1 if the algo converged 0 else
%        .Iteration : numer of iterations for convergence
%        .LastImprovement  : value of the last improvment
%        .Ariv :Matrix containing F coefficient  size( n_a,n_alpha)
%        .Briv :Matrix containing B coefficient  size( n_b,n_beta)
%        .Pinv :Covariance matrix on estimated coefficients
%  - ysim : output simulated using the estimated model
%  ======================================================================
% INPUTS:
%   -DATA: the output-input data  and an IDDATA object. See HELP IDDATA.
%    contains y and u (respective size (1,N) ) 
%   
%    -NN = [nb nf nk nc nd],  the orders and delays of the above model
%               The routine considers SISO systems only
%   -TOL     : tolerance on relative variations of the estimated parameters,
%  	          tol is used to stop the algorithm (default value: 1e-4)
%   -MAXITER : maximum of loops for the iterations (default value: 20)
%   -Ascheduling : Scheduling parameters size(n_alpha, N)
%   -Bscheduling: Scheduling parameters size(n_beta, N)
% 
%  OPTIONAL
%    BInit: Initialisation LPV Numerator size(n_b,n_beta)
%    FInit: Initialisation LPV Denominator size(n_a,n_alpha)
%==========================================================================
% ADDITIONAL COMMENTS
%
%  -An example code can be found in rivLPVaxample.m
%  - The estimated model can be simulated by calling the function
%     ysimulated = simulLPV(u, infos.Ariv, pA, infos.Briv, pB);  
%
%
%	See for further explanations :
%	Laurain et al,
%	"Refined Instrumental variables for identification of LPV models",
%	Automatica

%	author  : Vincent Laurain
%	date     : October 2009
%	revision : 
%  	name     : rivLPV.m
%
%	CRAN - Centre de Recherche en Automatique de Nancy
%	e-mail : vincent.laurain@cran.uhp-nancy.fr


%*** Preliminary calculations ***
if nargin<2, help sriv; error('too few input parameters in sriv'); end
if(~isa(data,'iddata')), error('data should be an iddata'); end

if nargin<3,	tol =1e-4;	end
if nargin<4,    maxiter=20; end

if isempty(tol)
    tol=1e-4;
end

if isempty(maxiter)
    maxiter=20;
end

%store the different orders of the system
nb=nn(1);
na=nn(2);
nk = nn(3);
nc = nn(4);
nd = nn(5);

%order of the 
ordera = size(pA,1);
orderb = size(pB,1);


Ts = data.Ts;
if(length(Ts)>1), error('Sampling period must be scalar'); end

% Processing of Input Delays
%data=remdelay(data,nk);

y = data.y;
u = data.u;
N=data.N;   % Number of data points


%nmax=max([na+1 nb+nk]);
nmax=max([na+1 nb+nk])-1;


%==========================================================================
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Initialisation%%%%%%%%%%%%%%%%%%%%
%==========================================================================

%if there is no initialisation of Ae and BE
if nargin<8
    
% First, estimate an ARX model and then use A_arx to prefilter the data
% *** construct regression matrix for the ARX model***
    phiy = [];phiu=[];

    for kl=1:na
        for order = 1:ordera
            phiy=[phiy -y(nmax+1-kl:N-kl,1).*pA(order,nmax+1:N)'];
        end
    end

    for kl=1:nb
        for order = 1:orderb
            phiu=[phiu u(nmax+2-kl-nk:N-kl-nk+1,1).*pB(order,nmax+1:N)'];
        end
    end
    Phiarx=[phiy phiu];
    % *** compute LS estimate ***
    [~,p]=size(Phiarx);

    AA = [Phiarx y(nmax+1:N,1)];
    Ro = triu(qr(AA));
    R1 = Ro(1:p,1:p);
    R2 = Ro(1:p,p+1);
    tharx = (R1 \ R2);

%Extract the matrix coefficients 
    Ae=[1 zeros(1,size(pA,1)-1)];
    for k =1:na
        Ae = [Ae; tharx((k-1)*ordera +1 : k*ordera)'];
    end

    Be =[];
    for k =1:nk
        Be=[Be ; zeros(1,size(pB,1))];
    end
    indexstartB = na *ordera;
    for k =1:nb
        Be = [Be; tharx(indexstartB + (k-1)*orderb +1  :indexstartB +  k*orderb)'];
    end
end
Ce =zeros(1,nc+1);
Ce(1) =1;
De =zeros(1,nd+1);
De(1) =1;


LastImprovement=2*tol;
ni=0;
apariv= 0;

%Compute the estimated output
yam = simulLPV(u,Ae,pA,Be,pB);
%check if the model is stable
if(max(abs(yam))>max(abs(y))*100 )
    %if not compute a stable model
    yam = simulLPVstab(u,Ae,pA,Be,pB);
end

%==========================================================================
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%start the iterations%%%%%%%%%%%%%%%%%%%%
%================================================================================
while(LastImprovement>tol && (ni<maxiter))

    % Take the nominal part of Ae and stabilize it
    At = Ae(:,1)';
    At=fstab(At);
    uf =[];

    %compute D/CF sche_i(t_k) * y(t_k-i)
    for kl=1:na
        for order = 1:ordera
            yuf(:,kl,order)=filter(1,At,y(nmax+1-kl:N-kl,1).*pA(order,nmax+1:N)');
            yuf(:,kl,order)=filter(De,Ce,yuf(:,kl,order));
        end
    end
    %compute D/CF sche_i(t_k) * u(t_k-i)
    for kl=1:nb
        for order = 1:orderb
            uf(:,kl,order) = filter(1,At,u(nmax+2-kl-nk:N-kl-nk+1,1).*pB(order,nmax+1:N)');
            uf(:,kl,order)=filter(De,Ce,uf(:,kl,order));

        end
    end

    %compute D/CF sche_i(t_k) * \hat y(t_k-i)
    yuamf =[];

    for kl=1:na
        for order = 1:ordera
            yuamf(:,kl,order)=filter(1,At,yam(nmax+1-kl:N-kl,1).*pA(order,nmax+1:N)');
            yuamf(:,kl,order)=filter(De,Ce,yuamf(:,kl,order));

        end
    end



%Build the regressor phi

    phiyf = [];phiuf=[];


    for kl=1:na
        for order = 1:ordera

            if order ==1
                phiyf=[phiyf -yuf(:,kl,order)];
            else
                phiyf=[phiyf -yuamf(:,kl,order)];
            end
        end
    end

    for kl=1:nb
        for order = 1:orderb
            phiuf=[phiuf uf(:,kl,order)];
        end
    end

    Phif=[phiyf phiuf];



%Build the instrument Psi


    psiyf=[];

    for kl=1:na
        for order = 1:ordera
            psiyf=[psiyf -yuamf(:,kl,order)];
        end
    end

    Psif=[psiyf phiuf];
    yf=filter(1,At,y);
    yf=filter(De,Ce,yf);
    
    % *** compute RIV estimate ***
    %thsriv=inv(Psif'*Phif)*Psif'*yf(nmax+1:N)
    [~,p]=size(Phif);
    AA = [Psif'*Phif Psif'*yf(nmax+1:N)];
    Ro = triu(qr(AA));
    R1 = Ro(1:p,1:p);
    R2 = Ro(1:p,p+1);
    thsriv = (R1 \ R2);

%check the conditioning of the matrices
    if rcond(R1)< 10^(-20) || isnan(rcond(R1))
       disp('results not stored');
       thsriv =0*thsriv;
       Ce =0*Ce;
       De =0*De;
       converge =0;
       Ariv=0*Ae;
       Briv=0*Be;
       % Send some arbitrary value if not storing results   
       infos.J		=	100000;  
       infos.converge=converge;
       infos.Iteration=100000;
       infos.LastImprovement = 100000;
       infos.Ariv = Ariv;
       infos.Briv = Briv;
       infos.Priv = 0;
       ysim =100000;
       return
    end
        

% Extract Ae and Be under matricial form for simulation
    Ae=[1 zeros(1,size(pA,1)-1)];
    for k =1:na
        Ae = [Ae; thsriv((k-1)*ordera +1 : k*ordera)'];
    end

    Be =[];
    for k =1:nk
        Be =[Be ; zeros(1,size(pB,1))];
    end
    indexstartB = na *ordera;
    for k =1:nb
        Be = [Be; thsriv(indexstartB + (k-1)*orderb +1  :indexstartB +  k*orderb)'];
    end



%simulate the estimated model
    yam = simulLPV(u,Ae,pA,Be,pB);
    %if not stable
    if(max(abs(yam))>max(abs(y))*100)
        
           %stabilize Ae by putting the maximum absolute root to 1 and
           %others accordingly
           Ae = myLPVstab2(Ae,pA);
            
           %recompute the Be corresponding to the New Ae (only B are to be
           %estimated)
            NewA =  Ae(2:end,:)';
            tosubtract = phiyf*NewA(:);
            newyf = yf(nmax+1:N) - tosubtract;
            [~,pt]=size(phiuf);

            AAt = [phiuf newyf];
            Rot = triu(qr(AAt));
            R1t = Rot(1:pt,1:pt);
            R2t = Rot(1:pt,pt+1);
            newB = (R1t \ R2t);
% Compute the new B
            Be =[];
            for k =1:nk
                Be =[Be ; zeros(1,size(pB,1))];
            end
            indexstartB = 0;
            for k =1:nb
                Be = [Be; newB(indexstartB + (k-1)*orderb +1  :indexstartB +  k*orderb)'];
            end
%resimulate the estimated system
            yam = simulLPV(u,Ae,pA,Be,pB);

    end

%compute the residuals
   resi = y-yam;

%estimate noise model
    datav=iddata(resi,[]);
    if (nd>0 || nc>0)
        % TODO: find out how to stop PEM from closing previous progressgui
        % object windows
        Mv=pem(datav,[nd nc]);
        Ce=Mv.C.';
        De=Mv.A.';
        De=fstab(De);
        Ce =fstab(Ce);

    else
        Ce=1;
        De=1;
    end


%Compute the last improvment
    DummyAe = Ae(2:end,:)';
    DummyBe = Be(nk+1:end,:)';

    LastImprovement=100*norm((apariv-[DummyAe(:)' DummyBe(:)' Ce(2:end)' De(2:end)'])./[DummyAe(:)' DummyBe(:)' Ce(2:end)' De(2:end)']);
    if isnan(LastImprovement)
        LastImprovement =100000;
    end

    apariv = [DummyAe(:)' DummyBe(:)' Ce(2:end)' De(2:end)'];

    %in case it does not converge it takes the value minimizing the
    %error on the output so it stores all results for all iterations
    thsrivnotconv(:,ni+1) = [DummyAe(:)' DummyBe(:)' ];
    Cenotconv(:,ni+1) = Ce';
    Denotconv(:,ni+1) = De';
    residuenotconv(ni+1) = sum( resi.^2); %#ok<*AGROW>
    Aenotconv{ni+1}=Ae;
    Benotconv{ni+1}=Be;

    ni=ni+1;
end 

%==========================================================================
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Final storing %%%%%%%%%%%%%%%%%%%%
%==========================================================================

%if no convergence then take the results minimizing the error on output
converge =1;
if (ni==maxiter)
    disp('did not converge');
    converge =0;
    indexmin = find(residuenotconv==min(residuenotconv));
    thsriv =  thsrivnotconv(:,indexmin) ;
    Ce = Cenotconv(:,indexmin);
    De = Denotconv(:,indexmin);
    Briv = Benotconv{indexmin};
    Ariv = Aenotconv{indexmin};
%otherwise take the last value
else
    thsriv = [DummyAe(:); DummyBe(:)];
    Briv =Be;
    Ariv =Ae;
end

%hand out the final simulated output and all informations 
   ysim = simulLPV(u,Ariv,pA,Briv,pB);
   e = ysim-y;
   e =filter(De,Ce,e);
   infos.J		=	e' * e / N ;  
   infos.converge=converge;
   infos.Iteration=ni;
   infos.LastImprovement = LastImprovement;
   infos.Ariv=Ariv;
   infos.Briv=Briv;

%==========================================================================
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Error on the parameters%%%%%%%%%%%%%%%%%%%%
%==========================================================================

    At = Ariv(:,1)';
    At=fstab(At);
    uf =[];



    for kl=1:na
        for order = 1:ordera
            yuf(:,kl,order)=filter(1,At,y(nmax+1-kl:N-kl,1).*pA(order,nmax+1:N)');
            yuf(:,kl,order)=filter(De,Ce,yuf(:,kl,order));
        end
    end
    for kl=1:nb
        for order = 1:orderb
            uf(:,kl,order) = filter(1,At,u(nmax+2-kl-nk:N-kl-nk+1,1).*pB(order,nmax+1:N)');
            uf(:,kl,order)=filter(De,Ce,uf(:,kl,order));

        end
    end

    yam=ysim;
    yuamf =[];



    for kl=1:na
        for order = 1:ordera
            yuamf(:,kl,order)=filter(1,At,yam(nmax+1-kl:N-kl,1).*pA(order,nmax+1:N)');
            yuamf(:,kl,order)=filter(De,Ce,yuamf(:,kl,order));

        end
    end
phiuf=[];
     for kl=1:nb
        for order = 1:orderb
            phiuf=[phiuf uf(:,kl,order)];
        end
    end


    psiyf=[];

    for kl=1:na
        for order = 1:ordera
            psiyf=[psiyf -yuamf(:,kl,order)];
        end
    end

    Psif=[psiyf phiuf];

infos.Priv=pinv(Psif'*Psif) * infos.J;
