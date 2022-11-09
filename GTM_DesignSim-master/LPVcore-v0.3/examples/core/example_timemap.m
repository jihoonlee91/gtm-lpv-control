% The timemap object keeps track of dynamic dependency and enables 
% the use of the derivative or shift operator for scheduling dependent 
% coefficients. The object creates a mapping between the scheduling signal 
% p(t) and the time-derivatives or time-shifts used in the pmatrix.
% The timemap object also functions as the core representation of a
% scheduling variable and hence it contains all relevant information about
% it.
%% Cerate a CT time map with static and dynamic dependence
T = timemap(0, 'CT');      % Static dependence on p(t)

%   Syntax:
%   obj = timemap;
%   obj = timemap(TimeMap);
%   obj = timemap(TimeMap,ContinuousTime);
%   obj = timemap(TimeMap,ContinuousTime,PV);
%
%   TimeMap 
%       The mapping of discrete time-shifts or continuous-time derivatives.
%       TimeMap is a vector of integers. For discrete-time maps the time
%       shifts can be any integer while for continuous-time maps, only
%       non-negative values are allowed. 
%
%   ContinuousTime 
%       Variable to indicate if the time map is continuous time or
%       discrete time. ContinuousTime = true or ContinuousTime = 'CT'
%       indicates continuous time, ContinuousTime = false or ContinuousTime
%       = 'DT' indicates discrete-time. Default ContinuousTime = false.

T = timemap([0 1 2], 'CT')   % Dynamic dependence on p(t), d/dt p(t), d^2/dt^2 p(t)

%% Create a DT time map with static and dynamic dependence

T = timemap(0, 'DT')      % Discrete dependence on p(t)

T = timemap([-2 0 1], 'DT')   % Dynamic dependence on p(t-2), p(t), p(t+1)


%% Additional properties
% Scheduling dimension:
T=timemap(0, 'DT', 'Dimension', 2)   % Creating two dimensional p(t)

% The dimension of the scheduling signal p(t). Should be a non-negative
% integer. This variable can be accessed by obj.np as well. Default:
% Dimension = 0.

% Naming of the scheduling variables
T = timemap(0,'DT','Dimension',2,'Name',{'postion','rotation'})

% Scheduling variable names, specified as one of the following:
%
% - Character vector: For a scalar scheduling signal. For example,
%   'velocity'. 
% - Cell array of character vectors: For multi-dimensional scheduling
%   signals.  
%         
% You can use the shorthand notation p to refer to the SchedulingName
% property. For example, sys.p is equivalent to sys.SchedulingName.
%         
% Default: {'p1','p2',...} for all scheduling channels.

% Specifying the units of the sceduling variables

T=timemap(0,'DT','Dimension',2,'Unit',{'m','rad'})
% Scheduling channel units, specified as one of the following:
%
% - Character vector: For a scalar scheduling signal. For example, 
%   'seconds'.
% - Cell array of character vectors: For multi-dimensional scheduling
%   signals.
%         
% Use SchedulingUnit to keep track of scheduling signal units.
% SchedulingUnit has no effect on behavior of system objects.
%         
% Default: '' for all scheduling channels

% Scheduling range
R={[-1 pi]; [2 pi]};
T=timemap(0,'DT','Dimension',2,'Range',R)
% Scheduling channel range, specified as a cell array of Dimension
% elements, where each element is a vector with 2 elements indicating the
% minimum and maximum range of the scheduling signal for that dimension.
% 
% Use SchedulingRange to specify the range for which the i-th scheduling
% variable is bound to, i.e., a_{i,min} <= p_i(t) <= a_{i,max}. Then 
%         SchedulingRange = [a_{1,min}, ... , a_{np,min}
%                            a_{1,max}, ... , a_{np,max} ];

%% Operations

T.Dimension       % Displays selected property

T.Range=R;         % Sets selected property

length(T)       % Gives the dimension of the time-map signal rho(t)

size(T)         % Gives the dimension of the time-map signal rho(t)
[mp,np]=size(T)     % Gives the dimension of the time-map signal rho(t) 
                    % and the dimension of the scheudling variable p(t) 
                    % in the time map
                    
rho=createRho(T,[sin(pi/2*(1:5));cos(pi/4*(1:5))]')  
% Evaluates the time map for a given trajectory of p(t)

shift(T,2)      % Shifts the discrete-time timemap by 2 smaples forward

T.Domain = 'ct';
shift(T,1)       % Differentiates the continuous-time timemap once


T1=timemap(0,'DT','Dimension',2,'Name',{'postion','rotation'});
T2=timemap(-1,'DT','Name',{'current'});

T= union(T1, T2);           % It is possible to fuse time maps by adding them together, 
                  % which corresponds to the set operation union.
%% References: 
%
% [1] Toth, R.: Modeling and identification of linear
%     parameter-varying systems. Lecture Notes in Control and Information
%     Sciences, Vol. 403, Springer, Heidelberg, 2010.



