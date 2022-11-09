%% In this example, we show the classes pmatrix and timemap.
%
%  With timemap, we create the time-map of a scheduling signal. This object
%  is required to take track of the derivative or shift operator for the
%  scheduling signal.
%
%  With pmatrix, we create the parameter-varying matrix functions.
%

%% Timemap object

% A time-map is by default discrete-time, consisting of only p(t)
timemap;

% To create the following discrete-time time-map: rho(t) =
% [p(t+5),p(t+1),p(t-2)]; use
timemap([5,1,-2], 'dt');

% To create the following discrete-time time-map: rho(t)=[p(t), d^2/dt^2
% p(t), d^1/dt^1 p(t)]; use
timemap([0,2,1], 'dt');

% timemap objects can be joined together using UNION. For example
union(timemap(5, 'dt'), timemap([1 -2], 'dt'))
% provides a discrete-time time-map: rho(t) = [p(t+5),p(t+1),p(t-2)];

% To see more info on timemap, type "help timemap"


%% Simple parameter varying variables

k = preal('k');
c = preal('c');

A = [0 1;0 0] + [0 0;1 0]*k+ [0 0;0 1]*c*c;
% Alternatively
A = [0 1;k c*c];


% Continuous time
k = preal('k', 'ct');
c = preal('c', 'ct');

A = [0 1;k c*c];


%% Pbasis object - static dependency

% To create a parameter-varying, we will provide some examples

% ------
% To create the following affine parameter-varying scalar 
%
%       A = 5 * rho(t)
%
% with rho(t)=p(t) of dimension 1, use:
%
pmatrix(5)

5*pmatrix(1,'SchedulingName',{'position_x'})


% ------
% To create the following affine parameter-varying matrix 
%   
%       A = [1 2] + [3 4] * rho(t,1) + [5 6] * rho(t,2)
%
% where m(t) = p(t) use
%
pmatrix( cat(3,[1 2], [3 4], [5 6]), 'SchedulingDimension', 2 )

% ------
% * To create the following affine parameter-varying matrix 
%   
%       A = [1 2] * rho(t,3) + [3 4] * rho(t,4) + [5 6] * rho(t,2)
%
% where m(t) = p(t) use
%
pmatrix( cat(3,[1 2], [3 4], [5 6]), ...
    'SchedulingDimension', 4, ...
    'BasisParametrization', {3, 4, 2} )

% ------
% To create the following polynomial parameter-varying matrix 
%   
%       A = [1 2] + [3 4] * rho(t,1) + [5 6] * rho(t,1)^2
%
% where m(t) = p(t) use
%
pmatrix( cat(3,[1 2], [3 4], [5 6]), ...
    'BasisType', 'poly')

% ------
% To create the following polynomial parameter-varying matrix 
%   
%       A = [1 2] + [3 4] * rho(t,1) + [5 6] * rho(t,1)^2
%
% where m(t) = p(t) use
%
pmatrix( cat(3,[1 2], [3 4], [5 6]), ...
    'BasisType', 'poly')

% ------
% To create the following polynomial parameter-varying matrix 
%   
%       A = [1 2]* rho(t,1)^2 * rho(t,2) + [3 4] * rho(t,1) * rho(t,2)^4 *
%           rho(t,3)^5 + [5 6]
%
% where m(t) = p(t) use
%
pmatrix( cat(3,[1 2], [3 4],[5 6]), ...
    'SchedulingDimension', 3, ...
    'BasisType', 'poly', ...
    'BasisParametrization',{[2, 1, 0], [1, 4, 5], [0, 0, 0]})

% ------
% To create the following (custom) parameter-varying matrix 
%   
%       A = [1 2]* cos(rho(:,2)) + [3 4] * sin(rho(:,1)).^2 + [5 6]
%
% where m(t) = p(t) use
%
pmatrix( cat(3,[1 2], [3 4], [5 6]), ...
    'BasisType', 'custom', ...
    'BasisParametrization', {@(rho)cos(rho(:,2)), @(rho)sin(rho(:,1)).^2, []}, ...
    'SchedulingDimension', 2)


%% Pbasis object - Dynamic dependency
%
% ------
% To create the following discrete-time, affine parameter-varying matrix 
%
%       A = [1 2]* p(t) + [3 4] * p(t-1) + [5 6] * p(t+2)               (2)
%
% we will define rho(t) = [p(t+2); p(t); p(t-1)] via timemap([2 0 -1]), and
% we substitute rho(t) in the latter equation, which gives
%
%       A = [1 2]* rho(t,2) + [3 4] * rho(t,3) + [5 6] * rho(t,1).
%
% Hence, Eq. (2) is obtained by using 
%
pmatrix( cat(3,[1 2], [3 4], [5 6]), ...
    'BasisParametrization', {2, 3, 1}, ...
    'SchedulingTimeMap',timemap([2 0 -1], 'dt'))

% ------
% To create the following discrete-time, affine parameter-varying matrix 
%
%       A = [1 2]* p(t,1) + [3 4] * p(t-1,1) + [5 6] * p(t-1,2)         (3)
%
% we will define rho(t) = [p(t); p(t-1)] via timemap([0 -1]), and we
% substitute rho(t) in the latter equation, which gives
%
%       A = [1 2]* rho(t,1) + [3 4] * rho(t,3) + [5 6] * rho(t,4).
%
% Hence, Eq. (3) is obtained by using 
%
pmatrix( cat(3,[1 2], [3 4], [5 6]), ...
    'BasisParametrization', {1, 3, 4}, ...
    'SchedulingTimeMap', timemap([0 -1], 'dt', 'Dimension', 2))

%% Pbasis object - plus, minus, times, and divide operations
A1 = pmatrix(rand(3,2,3), ...
    'BasisType', 'poly', 'BasisParametrization', {[1 2], [2, 2], [0, 0]}, ...
    'SchedulingDimension', 2);
A2 = pmatrix(rand(3,2,2), ...
    'BasisType', 'poly', 'BasisParametrization', {[1 2],[1, 2]}, ...
    'SchedulingDimension', 2);
A3 = pmatrix(rand(3,2), ...
    'BasisType', 'poly', 'BasisParametrization', {[1 1]}, ...
    'SchedulingDimension', 2);
A4 = pmatrix(rand(3,2,3), ...
    'BasisParametrization', {0, 1, 2}, 'SchedulingDimension', 2);
A5 = pmatrix(rand(3, 3), ...
    'BasisType', 'poly', 'BasisParametrization', {[2 1]}, ...
    'SchedulingDimension', 2);

%Valid operations:
-A1
A1 + A2
A1 - A2
5*A1
rand(3,3)*A1
A1.*A4

randn(3,4)\A1
randn(3,2).\A1
A1 / rand(1,2)
A1 ./ rand(3,2)

[A4,A3;A5,rand(3,1)];