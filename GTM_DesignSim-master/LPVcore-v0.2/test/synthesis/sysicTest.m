function tests = sysicTest
%SYNTHESISTEST Test synthesis algorithms
tests = functiontests(localfunctions);
end

function testSysic(testCase)
    rng(1);

    %% Variables
    p = preal('p','dt');
    Ts = randi([1 10]);

    %% Systems
    % G
    Ag = rand+rand*p;
    Bg1 = rand+rand*p; Bg2 = rand; Bg = [Bg1, Bg2];
    Cg = rand+rand*p;
    Dg1 = rand; Dg2 = rand; Dg = [Dg1, Dg2];

    G = lpvss(Ag,Bg,Cg,Dg,Ts);

    % W
    Aw = rand+rand*p;
    Bw = rand;
    Cw = rand+rand*p;
    Dw = rand;

    Wp = lpvss(Aw,Bw,Cw,Dw,Ts);

    %% Sysic interconnection
    systemnames     = 'G Wp';
    inputvar        = '[w;u(2)]';
    input_to_G      = '[u(1)+u(2);Wp]';
    input_to_Wp     = '[w+G]';
    outputvar       = '[Wp]';
    sysoutname      = 'P';
    sysic;

    % Useless operation such that MATLAB linting doesn't complain that 
    % variables being used by sysic are being 'unused'.
    sprintf([systemnames,inputvar,input_to_G,input_to_Wp,...
        outputvar,sysoutname,num2str(G.Ts),num2str(Wp.Ts)]);
    
    %% Manual interconnection
    den = 1/(1-Dw*Dg2);
    Ap = [Ag+Bg2*Dw*Cg*den,Bg2*Cw*den;Bw*(Dg2*Dw*Cg*den+Cg),Aw+Bw*Dg2*Cw*den];
    Bp = [Bg2*Dw*den,Bg1+Bg2*Dw*Dg1*den,Bg1+Bg2*Dw*Dg1*den;...
        Bw*(1+Dg2*Dw*den),Bw*(Dg1+Dg2*Dw*Dg1*den),Bw*(Dg1+Dg2*Dw*Dg1*den)];
    Cp = [Dw*Cg*den,Cw*den];
    Dp = [Dw*den,Dw*Dg1*den,Dw*Dg1*den];

    %% Check if equivalent
    % Matrices manual interconnection
    AP = Ap.matrices(:);
    BP = Bp.matrices(:);
    CP = Cp.matrices(:);
    DP = Dp(:);

    % Matrices sysic interconnection
    PA = P.A.matrices(:);
    PB = P.B.matrices(:);
    PC = P.C.matrices(:);
    PD = P.D.matrices(:);   PD(PD<=10*eps) = []; 
    
    % Verify
    sysicCheck = all([norm(AP-PA),norm(BP-PB),norm(CP-PC),norm(DP-PD)]<=10*eps);
    
    verifyTrue(testCase, sysicCheck);   
end