function disp(obj)
%DISP Display information

narginchk(1, 1);

if isempty(obj)
    fprintf('Empty identified LPV-OBF model');
    return;
end

if ~isempty(obj.Hf) && isempty(obj.Wf)
    type = 'Hammerstein';
    diagram = ['    +--------+   +-------+   e     \n', ...
               '    |        |   |       |   |     \n', ...
               'u-->|  Hf(p) +-->|  OBF  +-->+-->y \n', ...
               '    |        |   |       |         \n', ...
               '    +--------+   +-------+         \n\n'];
elseif ~isempty(obj.Wf) && isempty(obj.Hf)
    type = 'Wiener';
    diagram = ['    +-------+   +--------+   e     \n', ...
               '    |       |   |        |   |     \n', ...
               'u-->|  OBF  +-->|  Wf(p) +-->+-->y \n', ...
               '    |       |   |        |         \n', ...
               '    +-------+   +--------+         \n\n'];
elseif ~isempty(obj.Wf) && ~isempty(obj.Hf)
    type = 'Hammerstein-Wiener';
    diagram = ['    +--------+   +-------+   +--------+   e     \n', ...
               '    |        |   |       |   |        |   |     \n', ...
               'u-->|  Hf(p) +-->|  OBF  +-->|  Wf(p) +-->+-->y \n', ...
               '    |        |   |       |   |        |         \n', ...
               '    +--------+   +-------+   +--------+         \n\n'];
else
    type = '';
    diagram = ['    +-------+   e     \n', ...
               '    |       |   |     \n', ...
               'u-->|  OBF  +-->+-->y \n', ...
               '    |       |         \n', ...
               '    +-------+         \n\n'];
end

if isct(obj)
    timeStr = 'Continuous-time';
else
    timeStr = 'Discrete-time';
end

fprintf('%s identified LPV-OBF %s model\n\n', timeStr, type);
fprintf(diagram);

% Description of input-scheduling-output
fprintf('with\n\n');
if ~isempty(obj.Hf)
    fprintf('    Hf:        %d-by-%d scheduling dependent matrix function\n', ...
        size(obj.Hf, 1), size(obj.Hf, 2));
end
    fprintf('    OBF bank:  %i basis functions\n', ...
        size(obj.A, 1) + 1);  % "+1" because of feedthrough
if ~isempty(obj.Wf)
    fprintf('    Wf:        %d-by-%d scheduling dependent matrix function\n', ...
        size(obj.Wf, 1), size(obj.Wf, 2));
end
fprintf('\nand\n\n');
fprintf('    Nr. of inputs:             %d\n', obj.Nu);
fprintf('    Nr. of scheduling signals: %d (extended: %d)\n', obj.Np, obj.Nrho);
fprintf('    Nr. of outputs:            %d\n\n', obj.Ny);

% Sample time
if obj.Ts == -1; TsStr = 'unspecified';
else;            TsStr = sprintf('%g %s',obj.Ts,obj.TimeUnit);                      end

fprintf('Sampling time: %s\n', TsStr);

fprintf('\nParameterization:\n');
if obj.D ~=0; fprintf('   Feedthrough: yes\n'); else; fprintf('   Feedthrough: no\n'); end
fprintf('   Number of free coefficients: %d\n',nparams(obj, 'free'));
fprintf('\nStatus:\n%s\n',obj.Report.Status);

end