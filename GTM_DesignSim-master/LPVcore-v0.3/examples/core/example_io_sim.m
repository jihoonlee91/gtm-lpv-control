%% In this example, we show the classes lpvio and lpvidpoly.
clearvars; close all; clc;

%% LPV-IO object

% Create the parameter-varying matrices
p1 = preal('pos', 'DT');
p2 = preal('vel', 'DT', 'Dynamic', -1);
p3 = preal('pos', 'DT', 'Dynamic', -2);
 
ny = 2;  nu = 3; B10 = randn(ny,nu);
a1 = diag([-0.5,-0.55])-0.1*p1;
a2 = diag([0.3,0.35])  +0.2*p2;
b1 = B10+0.1*p1+0.2*p3;
b2 = [diag([-0.25,-0.25]),[0;0]]*p1+[[0;0],diag([0.1,0.15])]*p3;

% Create LPV-IO object
sys = lpvio({eye(2)}, {b1,b2});
sys.InputName = {'Blue Frame', 'Red Frame','Silver Frame'};

size(sys)
ndims(sys)

% Simulate LPV-IO object, y(t) = B(q,p(t)) u(t).
N = 400;
u = randn(N,nu);
p = [sin((1:N)*0.1)', sin((1:N)*0.2)']; 

lsim(sys,p(:,1),u);

% Simulate LPV-IO object, A(q,p(t)) y(t).
sys = lpvio({eye(2), a1, a2}, {});
lsim(sys, p(1:50,:), [], ...
    'y0', 20 * randn(2, 2), ...
    'p0', 5 * randn(1, 2));
%    'InitialCondition', {20*randn(2,2), 5*randn(1,2),[]})

% Simulate LPV-IO object, A(q,p(t)) y(t) = B(q,p(t)) u(t).
sys = lpvio({eye(2), a1, a2}, {b1, b2});
[y,t] = lsim(sys,p,u);
figure; plot(t,y); legend('y1','y2');

%% LPV-IDPOLY object
% Create the parameter-varying matrices
p1 = preal('pos', 'DT');
p2 = preal('vel', 'DT', 'Dynamic', -1);
p3 = preal('pos', 'DT', 'Dynamic', -2);
 
ny = 2;  nu = 3; C10 = rand(ny,ny); C20 = rand(ny,ny); B10 = randn(ny,nu);
a1 = diag([-0.5,-0.55])-0.1*p1;
a2 = diag([0.3,0.35])  +0.2*p2;
c1 = C10+0.15*p1;
c2 = C20-0.35*p2;
b1 = B10+0.1*p1+0.2*p3;
b2 = [diag([-0.25,-0.25]),[0;0]]*p1+[[0;0],diag([0.1,0.15])]*p3;

% Create LPV-IDPOLY object. A(q,p(t)) y(t) =  C(q,p(t)) e(t).
sys = lpvidpoly({eye(2), a1, a2}, {}, {eye(2), c1, c2});
size(sys)
ndims(sys)

% Simulate LPV-IO object,
N = 400;
u = randn(N,nu);
p = [sin((1:N)*0.1)', sin((1:N)*0.2)']; 
%%
lsim(sys, p, []);

% Create LPV-IDPOLY object. A(q,p(t)) y(t) = B(q,p(t)) u(t) + C(q,p(t)) e(t).
sys = lpvidpoly({eye(2), a1, a2},{b1,b2},{eye(2), c1, c2});
lsim(sys,p,u);

    % Create LPV-IDPOLY object.  y(t) = F(q,p(t)) \ B(q,p(t)) u(t) + D(q,p(t)) \C(q,p(t)) e(t).
    %            A     B            C                   D               F
    %           ______ ________   __________________   ______________  ____________
sys = lpvidpoly(eye(2), {b1, b2}, {eye(2), C10,c1,c2}, {eye(2),a1,c2}, {eye(2), a1, a2});
lsim(sys,p,u);
