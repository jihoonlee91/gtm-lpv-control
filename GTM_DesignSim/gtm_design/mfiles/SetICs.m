function [StatesInp] = SetICs(ini)
StatesInp= zeros(12,1);

knots2fps = 1/0.592487;
d2r=pi/180;

ub = (knots2fps)*ini.tas*cos(ini.alpha*d2r);
wb = (knots2fps)*ini.tas*sin(ini.alpha*d2r);

theta    = ini.alpha + ini.gamma;

StatesInp(1)  = ub;              %  1 - ub (ft/s)
StatesInp(2)  = 0;		 %  2 - vb (ft/s)
StatesInp(3)  = wb;		 %  3 - wb (ft/s)
StatesInp(4)  = 0;		 %  4 - pb (rad/s)
StatesInp(5)  = 0;		 %  5 - qb (rad/s)
StatesInp(6)  = 0;		 %  6 - rb (rad/s)
StatesInp(7)  = ini.lat*d2r;	 %  7 - lat (rad), +north,
StatesInp(8)  = ini.lon*d2r;	 %  8 - lon (rad), +east,
StatesInp(9)  = ini.Altitude;	 %  9 - h (ft)
StatesInp(10) = 0;	             % 10 - phi (rad)
StatesInp(11) = theta*d2r;       % 11 - theta (rad)
StatesInp(12) = 0;            % 12 - psi (rad)

