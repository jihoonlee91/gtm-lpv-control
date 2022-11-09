function disp(obj)
% DISP Provide info on the lpvlfr object
%
%   Syntax:
%       disp(sys)
%
%   Inputs:
%       sys: lpvlfr object
%
narginchk(1, 1);

if isempty(obj)
    fprintf('Empty LPV-LFR model');
    return;
end

% Model type
if isct(obj)
    timeStr = 'Continuous-time';
else
    timeStr = 'Discrete-time';
end
fprintf('%s LPV-LFR model\n\n', timeStr);

% Block diagram
fprintf(['             +---------+          \n', ...
         '       +-----+ Delta(p)+<---+     \n', ...
         '       |     +---------+    |     \n', ...
         '     w |                    | z   \n', ...
         '       |   +------------+   |     \n', ...
         '       +-->+     G      +---+     \n', ...
         '  u +----->+            +------> y\n', ...
         '           +------------+         \n\n']);

% Description of input-scheduling-output
fprintf('with\n\n');
fprintf('    Delta: %d-by-%d scheduling dependent matrix function with %d basis function(s)\n', ...
    size(obj.Delta, 1), size(obj.Delta, 2), size(obj.Delta, 3));
fprintf('    G:     %d-by-%d %s LTI model with %d state(s)\n\n', ...
    size(obj.G, 1), size(obj.G, 2), lower(timeStr), obj.Nx);
fprintf('and\n\n');
fprintf('    Nr. of inputs:             %d\n', obj.Nu);
fprintf('    Nr. of scheduling signals: %d (extended: %d)\n', obj.Np, obj.Nrho);
fprintf('    Nr. of outputs:            %d\n', obj.Ny);
fprintf('\n');

% Sample time
if ~isct(obj)
    if obj.Ts == -1; TsStr = 'unspecified';
    else;            TsStr = sprintf('%g %s',obj.Ts,obj.TimeUnit);                      end
    
    fprintf('%s: %s.\n', 'Sample time', TsStr);
end

end
