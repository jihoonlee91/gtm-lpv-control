%% LPV-LFR modeling and simulation
%
% LPVcore supports Linear Parameter-Varying Linear-Fractional (LPV-LFR)
% system representations. These representations are based on the following LFT
% interconnection of an LTI system (G) and a scheduling-dependent matrix
% function (Delta):
%
%              +---------+          
%        +-----+ Delta(p)+<---+     
%        |     +---------+    |     
%      w |                    | z   
%        |   +------------+   |     
%        +-->+     G      +---+     
%   u +----->+            +------> y
%            +------------+  
%
% Such an interconnection admit a more flexible scheduling dependence
% compared to LPV-SS representations (lpvss objects). In fact, the lpvss
% object is implemented as a so-called child class of the lpvlfr object,
% which means that most functionality available for lpvlfr objects is
% directly available for lpvss objects. 
%
% To manipulate lpvlfr objects, LPVcore provides a wide range of functions.
% These functions will be demonstrated in this example script.
clearvars; clc; close all; rng(1);

%% Create LPV-LFR models
% Two lpvlfr objects will be created. First, the scheduling signals will be
% defined (in discrete-time, 'dt', although continuous-time, 'ct', is also
% possible).
a = preal('a', 'dt');
b = preal('b', 'dt');
c = preal('c', 'dt');

% Define the signal dimensions (see drawing on the top of this documents
% for the various signals defined for lpvlfr objects)
nx = 10;  % State dimension of LTI part
nu = 2;
ny = 2;
nw = 2;
nz = 2;

% Create the LTI part of the lpvlfr objects. Note the signal dimensions
% must be compatible with the LFR operation to be applied.
G1 = drss(nx, ny + nz, nu + nw);
G2 = drss(nx, ny + nz, nu + nw);
G1.Ts = 1;
G2.Ts = 1;

% Create the scheduling-dependent matrix functions (pmatrix objects)
D1 = a * randn(nw, nz) + b * randn(nw, nz);
D2 = c * randn(nw, nz) + b * randn(nw, nz);

sys1 = lpvlfr(D1, G1);
sys2 = lpvlfr(D2, G2);

%% Simulating lpvlfr objects
% Simulating lpvlfr objects is done with the "lsim" command (similar to the
% lpvio and lpvss objects). In addition to specifying the input signal, the
% scheduling signal must also be specified.
%
% First, we define the number of time steps and signals to excite the
% system.
N = 100;  % time steps
p = 0.01 * randn(N, sys1.Np);  % scheduling signal
u = randn(N, sys1.Nu);  % input signal
[y,t] = lsim(sys1, p, u);

% Display results
figure;
plot(t, y);
xlabel('Time (seconds)');
ylabel('Amplitude');
title('LPV Simulation Results');
            
%% Interconnecting lpvlfr objects
% Two or more lpvlfr objects can be combined in different ways that are
% available through intuitive commands.

% Example 1: parallel interconnection
%         ┌──────┐
%         │      │
% ───┬───►│ sys1 ├──────►+───►
%    │    │      │       ▲
%    │    └──────┘       │
%    │                   │
%    │    ┌──────┐       │
%    │    │      │       │
%    └───►│ sys2 ├───────┘
%         │      │
%         └──────┘
% To define a parallel interconnection, simply use the "+" operator.
sysparallel = sys1 + sys2;

% Example 2: serial interconnection
%     ┌──────┐   ┌──────┐
%     │      │   │      │
% ───►│ sys1 ├──►│ sys2 ├───►
%     │      │   │      │
%     └──────┘   └──────┘
% To define a serial interconnection, simply use the "*" operator.
sysserial = sys2 * sys1;

% Example 3: block-diagonal interconnection
%   ┌────────────────────────┐
%   │                        │
%   │  ┌────────┐            │
%   │  │        │            │
% ──┼─►│  sys1  ├────────────┼─►
%   │  │        │            │
%   │  └────────┘            │
%   │            ┌────────┐  │
%   │            │        │  │
% ──┼───────────►│  sys2  ├──┼─►
%   │            │        │  │
%   │            └────────┘  │
%   │                        │
%   └────────────────────────┘
% To define a block diagonal interconnection, use the "blkdiag" or "append"
% commands (both are equivalent).
sysblkdiag = blkdiag(sys1, sys2);

% Example 4: horizontal concatenation
%       ┌──────┐
%       │      │
% ─────►│ sys1 ├──────►+───►
%       │      │       ▲
%       └──────┘       │
%                      │
%       ┌──────┐       │
%       │      │       │
% ─────►│ sys2 ├───────┘
%       │      │
%       └──────┘
% Sum the output of both systems (keeping the inputs separate). This is a
% horizontal concatation and can be performed using bracket-notation
% ([sys1, sys2]).
syshorz = [sys1, sys2];

% Example 5: vertical concatenation
%          ┌──────┐
%          │      │
% ───┬────►│ sys1 ├────►
%    │     │      │
%    │     └──────┘
%    │
%    │     ┌──────┐
%    │     │      │
%    └────►│ sys2 ├────►
%          │      │
%          └──────┘
% Feed the same input to both systems (keeping the outputs separate). This
% is a vertical concatenation and can be performed using bracket-notation
% ([sys1; sys2]).
sysvert = [sys1; sys2];

% Example 6: general interconnection
% The most general interconnection command available is "connect". The user
% can specify which outputs of which systems should be connected to which
% inputs of which systems. The most convenient way to use it is name-based.
% This means that the interconnection is performed automatically by
% matching input and output names of the system.
%
% In the example, the following signal names will be used.
%
%       ┌──────┐
%       │      │
% u ───►│ sys1 ├────► y
%       │      │
%       └──────┘
%       ┌──────┐
%       │      │
% y ───►│ sys2 ├────► x
%       │      │
%       └──────┘
% Note that for vector-valued signals, the names are automatically
% expanded, i.e., 'u' becomes {'u(1)', 'u(2)', ..., 'u(nu)'}.
sys1.InputName = 'u';
sys2.InputName = 'y';
sys1.OutputName = 'y';
sys2.OutputName = 'x';
% Calling "connect" will connect the two signals both called "y".
% Besides the systems considered for the connection operation, we must also
% specify the final inputs and outputs. For the input, we select "u". For
% the output, we select "x" and also the first component of "y".
sysconn = connect(sys1, sys2, ...
    'u', {'y(1)', 'x'});
% Confirm the input and output signals
disp(sysconn.InputName');  % --> {'u(1)', 'u(2)'}
disp(sysconn.OutputName.'); % --> {'y(1)', 'x(1)', 'x(2)'}

