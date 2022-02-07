function [f,g]=simio(xu,Xo,Index,target)
%function [f,g]=gtmio(xu,Xo,Index,target,TrimWithStab)
%
% Calls model simulation in "precompiled" mode
% sets up objective function and constraints for optimization

%
% Setup tweaked vector
Xtweak=Xo;
Xtweak(Index.EOM(1:6))   = xu(1:6);        % [u;v;w;p;q;r]
Xtweak(Index.EOM(10:12)) = xu(7:9);        % [phi;theta;psi]      

Utweak           = zeros(23,1);
Utweak(1)        = xu(10);   % PitchSurf (Elevator or Stab)
Utweak(2)        = xu(11);  % Aileron 
Utweak(3)        = xu(12);  % Rudder 
Utweak(4)        = xu(13);  % Engine 

% Calculate outputs and derivatives
trim_at_time=1e-3; % sec, Avoiding zero as it might trigger IC blocks or
% other initialization procedures.
yout = gtm_design(trim_at_time,Xtweak,Utweak,'outputs');
dx   = gtm_design(trim_at_time,Xtweak,Utweak,'derivs');

% Form trim error on auxilary variables
delta=zeros(15,1);
if isfield(target,'eas'),      delta(1)  =yout(1)-target.eas;     end % eas
if isfield(target,'tas'),      delta(2)  =yout(2)-target.tas;    end % tash
if isfield(target,'alpha'),    delta(3)  =yout(3)-target.alpha;   end % alpha
if isfield(target,'beta'),     delta(4)  =yout(4)-target.beta;   end % beta
if isfield(target,'gamma'),    delta(5)  =yout(5)-target.gamma;    end % gamma
if isfield(target,'gndtrack'), delta(6)  =yout(6)-target.gndtrack;end % ground track
if isfield(target,'roll'),     delta(7)  =Xtweak(Index.EOM(10))-target.roll*pi/180;end   % roll
if isfield(target,'pitch'),    delta(8)  =Xtweak(Index.EOM(11))-target.pitch*pi/180;end   % pitch
if isfield(target,'yaw'),      delta(9)  =Xtweak(Index.EOM(12))-target.yaw*pi/180;end   % yaw (heading)
if isfield(target,'rollrate'), delta(10) =dx(Index.EOM(10))-target.rollrate*pi/180;  end  % phidot
if isfield(target,'pitchrate'),delta(11) =dx(Index.EOM(11))-target.pitchrate*pi/180; end  % thetadot
if isfield(target,'yawrate'),  delta(12) =dx(Index.EOM(12))-target.yawrate*pi/180;   end  % psidot
if isfield(target,'pdeg'),     delta(13) =Xtweak(Index.EOM(4))-target.pdeg*pi/180; end  % p
if isfield(target,'qdeg'),     delta(14) =Xtweak(Index.EOM(5))-target.qdeg*pi/180; end  % q
if isfield(target,'rdeg'),     delta(15) =Xtweak(Index.EOM(6))-target.rdeg*pi/180; end  % r

f=xu(end);  %Lagrange Multiplier
g=[dx(Index.EOM(1:6)); delta(:)-f; -delta(:)-f ]; % d/dt[uwv,pqr]=0 equality constraint,
% minimize trim error with Lagrange multiplier
end




