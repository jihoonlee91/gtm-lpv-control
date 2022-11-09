function [ySys, fit] = compare(data, varargin)
%COMPARE Compare identified model output and measured output
%
%   Syntax:
%       compare(data, sys, LineSpec)
%       compare(data, sys, LineSpec, kstep)
%       compare(data, sys, kstep)
%       compare(data, sys1, sys2, ..., sysn, ___)
%       [y, fit] = compare(data, sys, kstep)
%
%   Inputs:
%       data: lpviddata object with the measured input-scheduling-output
%       data
%       sys, sys1, ..., sysn: identified lpvidpoly system to compare with data
%       kstep: number of timesteps to predict the output. If left out, or
%           set to Inf, then the measured output is not used to do the
%           comparison.
%
%   Outputs:
%       y: simulated or predicted output response.
%       fit: fit in percentage between simulated/predicted and measured output
%           response, based on Normalized Root Mean Square Error (NRMSE)
%           between them.
%
kstep = [];
syss = {};  % Cell array of systems
sysNames = {};
lineSpecs = {};  % Line specifications
i = 1;
while i <= numel(varargin)
    v = varargin{i};
    if isa(v, 'lpvrep')
        syss = [syss(:)', {v}];
        sysNames = [sysNames(:)', {inputname(i+1)}];
        % Next argument may be a LineSpec
        if i < numel(varargin) && ischar(varargin{i+1})
            lineSpecs = [lineSpecs(:)', varargin(i+1)];
            i = i + 1;
        else
            lineSpecs = [lineSpecs(:)', {NaN}];
        end
    elseif isint(v) || (isscalar(v) && isinf(v))
        assert(i == numel(varargin), 'kstep must be last argument');
        kstep = v;
        break;
    else
        error('Invalid argument: %s\n', inputname(i+1));
    end
    i = i + 1;
end

p = data.p;
u = data.u;
y = data.y;
t = data.SamplingInstants;

% Prediction or simulation
if isempty(kstep) || isinf(kstep)
    isPred = false;
else
    isPred = true;
end

% Simulate or predict response
ySys = cell(numel(syss), 1);
fit = cell(numel(syss), 1);
for i=1:numel(syss)
    sys = syss{i};
    if ~isPred
        % Simulation
        if isa(sys, 'lpvidsys')
            % Get the deterministic output (4th)
            [~, ~, ~, ySys{i}] = lsim(sys, p, u, t);
        elseif isa(sys, 'lpvlfr')
            ySys{i} = lsim(sys, p, u, t);
        else
            error('Unexpected error occured');
        end
    else
        % Prediction
        if ~isa(sys, 'lpvidsys')
            error(['Input ', sysNames{i}, 'is not a model with identifiable parameters. ', ...
                'This is not supported at the moment for kstep = 1']);
        end
        ySys{i} = predict(sys, data, kstep);
    end
    % Calculate fit
    fit{i} = 100 * (1 - LPVcore.goodnessOfFit(ySys{i}, y, 'NRMSE'));
end

if nargout == 0
    % Show plot
    ny = sys.Ny;
    tile = tiledlayout(ny, 1);
    for i=1:ny
        ax(i) = nexttile; %#ok<AGROW>
        plot(t, y(:, i), 'k'); hold on;
        for j=1:numel(syss)
            tmp = ySys{j};
            if ~isnan(lineSpecs{j})
                plot(t, tmp(:, i), lineSpecs{j});
            else
                plot(t, tmp(:, i));
            end
        end
        if ~isempty(data.OutputName{i})
            outputName = data.OutputName{i};
        else
            outputName = ['y', int2str(i)];
        end
        % Build legend
        legendEntries = cell(1, numel(syss));
        legendEntries{1} = sprintf('%s (%s)', inputname(1), outputName);
        for j=1:numel(syss)
            f = fit{j};
            legendEntries{j+1} = sprintf('%s: %.2f%%', ...
                sysNames{j}, f(i));
        end
        legend(legendEntries, 'interpreter', 'none'); hold off;
        ylabel(outputName);
    end
    % Share x-axis between plots
    linkaxes(ax, 'x');
    % Format figure
    tile.TileSpacing = 'compact';
    if isPred
        type = [int2str(kstep), '-step Predicted'];
    else
        type = 'Simulated';
    end
    title(tile, [type, ' Response Comparison']);
    xlabel(tile, sprintf('Time (%s)', data.TimeUnit));
    ylabel(tile, 'Amplitude');
end

% Convert single-model output arguments from cell array
if numel(syss) == 1
    ySys = ySys{1};
    fit = fit{1};
end

end

