%% In this example, we show the lpviddata class
%
%  This example requires to set the current working directory to the
%  root of the toolbox.

clear all; close all; clc;

if ~exist('pmatrix','class'); cd(['..',filesep]); end  % Toolbox is not added to mathlab search path. Go to root of the toolbox


%% Simulate a system in LPV-ARX form

    % Create the parameter-varying matrices
p1 = preal('pos', 'DT');
p2 = preal('vel', 'DT', 'Dynamic', -1);
p3 = preal('pos', 'DT', 'Dynamic', -2);


a1 = diag([-0.5,-0.55])-0.1*p1;
a2 = diag([0.3,0.35])  +0.2*p2;
b1 = randn(2,3)+0.1*p1+0.2*p3;
b2 = [diag([-0.25,-0.25]),[0;0]]*p1+[[0;0],diag([0.1,0.15])]*p3;


sys = lpvidpoly({eye(2), a1,a2},{b1,b2});
    % Generate input and scheduling signals
N = 400;
u = randn(N,3);
p = [sin((1:N)*0.1)', sin((1:N)*0.2)']; 

y = lsim(sys,p,u);



%% Construct the lpviddata object


data = lpviddata(y,p,u,0.1,'InputName','motor','TimeUnit','minutes');

data = set(data,'SchedulingName',{'Blue frame','Red frame'}, 'SchedulingUnit', 'rad/s', 'Tstart', 5)
get(data)

[N,ny,np,nu] = size(data);
length(data)
ndims(data)


    % Plotting the first 50 time samples
dataSmall = data(1:50);
plot(dataSmall);

    % Plotting the last 50 time samples of y1, p2 only

datay = data(end-49:end, 1, 2, []);
plot(datay);



%% Removing offset in dataset

OutputOffset     = [-5 5];
SchedulingOffset = [];
InputOffset      = -1;  % remove -1 from each signal

dataOff = rmOffset(data,OutputOffset,SchedulingOffset,InputOffset);
plot(dataOff);