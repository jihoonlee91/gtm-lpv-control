function [sys,x0,str,ts] = sfunEvalMat(~,x,u,flag,M,nm,nn,np)
%SFUNEVALMAT Returns the evaluation of pmatrix M at the input u.

switch flag
    % Initialization 
    case 0
        [sys,x0,str,ts] = mdlInitialize(nm,nn,np);    

    % Output
    case 3                                             
        sys = mdlOutputs(x,u,M);    

    % Terminate 
    case {1,2,9}
        sys = [];

    otherwise
        error(['unhandled flag = ',num2str(flag)]);
end

%% Initialize
function [sys,x0,str,ts]=mdlInitialize(nm,nn,np)
sizes = simsizes;
sizes.NumDiscStates  = 0;
sizes.NumContStates  = 0;
sizes.NumInputs      = max(1,np);
sizes.NumOutputs     = nm*nn;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

x0  = [];
str = [];
ts  = [0 0];

%% Output
function sys = mdlOutputs(~,u,M)
p = u;
sys = reshape(freeze(M,p'),[],1);