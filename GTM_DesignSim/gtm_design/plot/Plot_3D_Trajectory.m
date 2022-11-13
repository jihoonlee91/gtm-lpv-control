%%

set(groot,'DefaultAxesFontSize',10)

d2r = pi/180; r2d = 180/pi;

t =  simOut.get('t');
tmax = t(end); dt = t(2)-t(1);
cmdPilot = simOut.get('cmdPilot'); % [deg/s] [g] [g]
cmdRate = simOut.get('cmdRate'); % [rad/s] [rad/s] [rad/s]
pseudoCont = simOut.get('pseudoCont');
cmdAngle = simOut.get('cmdAngle');
delc = reshape(simOut.get('delc'),[4 tmax/dt+1]).';
del = reshape(simOut.get('del'),[4 tmax/dt+1]).';
state16 = reshape(simOut.get('state16'),[16 tmax/dt+1]).';
accel = simOut.get('accel');
fm = reshape(simOut.get('fm'),[6 tmax/dt+1]).';
eta = simOut.get('eta');
quat = state16(:,7:10);
pos = state16(:,11:13);
[ps,th,ph] = quat2angle(quat);
[~,SoS,~,~] = atmosisa(-state16(:,13));
SoS = SoS.';

%%

figure('position',[300 300 560 465])

x = pos(:,1);
y = pos(:,2);
z = -pos(:,3);
pitch = th;
roll = ph;
yaw = -ps+pi/2;
scale_factor = 1/250;
step = 2.5*(length(t)-1)/t(end);
selector = 'mig';
varargin = [];

trajectory3(y,x,z,th,ph,-ps+pi/2,1/250,2.5*(length(t)-1)/t(end),'mig');

xlabel('East, m')
ylabel('North, m')
zlabel('Altitude, m')
view(37.5,30)
box on
% xlim([-500 2500])
% ylim([-500 3000])
% legend('max span','min span','location','northeast')