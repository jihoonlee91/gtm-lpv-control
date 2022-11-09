function plotlsim(sys, y, p, u, t)
%PLOTLSIM Plot results of LSIM when invoked without output arguments
%
%   Syntax:
%       plotlsim(sys, y, p, u, t)
%
%   Inputs:
%       sys: LPVREP system
%       y: N-by-Ny matrix
%       p: N-by-Np matrix
%       u: N-by-Nu matrix
%       t: N-by-1 matrix
%

Ny = sys.Ny;
Np = sys.Np;
Nu = sys.Nu;
% Three tiles for input, scheduling and output
tile = tiledlayout((Nu > 0) + (Np > 0) + (Ny > 0), 1);

% Input
if Nu > 0
    ax(1) = nexttile;
    plot(t, u);
    title('Input');
end
% Scheduling
if Np > 0
    ax((Nu > 0) + 1) = nexttile;
    plot(t, p);
    title('Scheduling');
end
% Output
if Ny > 0
    ax((Nu > 0) + (Np > 0) + 1) = nexttile;
    plot(t, y);
    title('Output');
end

% Share x-axis between plots
linkaxes(ax, 'x');

% Determine figure title based on system type
if isa(sys, 'lpvss')
    type = 'LPV-SS';
elseif isa(sys, 'lpvlfr')
    type = 'LPV-LFR';
elseif isa(sys, 'lpvio')
    type = 'LPV-IO';
elseif isa(sys, 'lpvidpoly')
    type = 'LPVIDPOLY';
elseif isa(sys, 'lpvidss')
    type = 'LPVIDSS';
else
    type = 'LPV';
end

% Add time domain
if isct(sys)
    type = ['Continuous-Time ', type];
else
    type = ['Discrete-Time ', type];
end

% Format figure
tile.TileSpacing = 'compact';
title(tile, [type, ' Simulation Results']);
xlabel(tile, 'Time (seconds)');
ylabel(tile, 'Amplitude');

end

