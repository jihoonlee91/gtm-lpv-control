function y = layer1atm(z,dt)
% Author: Kevin Cunningham          April 20, 1992    
% Modified: Dave Cox                Aug, 14, 2008

y=zeros(6,1);

%dt=0;  % Standard Temp forced.

%    --- constants from 1976 std. atmosphere
r     = 1716.55915670803;
gamma = 1.4;
g     = 32.17405;
re    = 20855531.5;
tsl   = 288.15 * 1.8;
psl   = 2116.22;
hsl   = 0.;
sldens = 0.0023769;                 % atmospheric density at sea level
slpr   = 2116.220;                  % sea level pressure, psf

%---------------------------------------------------------------------
%    conversion from metric lapse rate to english lapse rate
%    meters/0.3048 = ft
%    Kelvin * 1.8 = Rankine
rm2ft = 1000.0/0.3048;
conlapse = 1.8/rm2ft;

%  ---- convert geometric alt. to geopotential alt.
h     = (re*z)/(re+z);
 
%  ---- ensure we are in layer one (based on geopotential alt)
if (h > 11*rm2ft) % 11 km
  %fprintf(1,'Error, atmosphere model out of range %f, Limiting...\n',h);
  h=11*rm2ft;
end
if (h<0)
  %fprintf(1,'Error, atmosphere model out of range %f, Limiting...\n',h);
  h=0;
end

a1     = -6.5*conlapse;
t1     = tsl + a1*(h-hsl);
del1   = (t1/tsl)^(-g/r/a1);
p1     = del1 * psl;

%   ---- apply temperature increment from standard day*/
t1     = t1 + dt;

%   ---- calculate density from eqn. of state w/ corrected temp.*/
rho1   = p1 / (r * t1);
 
%   ---- calculate speed of sound w/ corrected temp.*/
c1     = sqrt(gamma * (r * t1));
 

% Calculate outputs
y(1) = rho1;
y(2) = p1;
y(3) = t1 / 1.8; % output temperature  in Kelvin
y(4) = c1;
y(5) = y(2) / slpr;         % pressure ratio
y(6) = sqrt(y(1)/sldens);   % SQUARE ROOT OF DENSITY RATIO

