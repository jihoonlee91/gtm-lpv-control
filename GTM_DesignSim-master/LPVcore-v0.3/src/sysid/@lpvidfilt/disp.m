function disp(obj)
%DISP Display for lpvidfilt objects

narginchk(1, 1);

if isempty(obj)
    fprintf('Empty lpvidfilt');
    return;
end

if isct(obj)
    Qstr  = '(d/dt)';
    Qistr = '(d/dt)^i';
else
    Qstr  = '(q^-1)';
    Qistr = '(q^-i)';
end

% Type (monic or not)
if ismonic(obj)
    typeStr = 'Monic polynomial';
else
    typeStr = 'Polynomial';
end

% Domain
if isct(obj)
    timeStr = 'Continuous-time';
else
    timeStr = 'Discrete-time';
end
fprintf('%s in %s (%s) with identifiable scheduling dependent coefficients:\n', typeStr, Qstr, lower(timeStr));

s = '';
for i=1:obj.N
    if numel(obj.Mat{i}.bfuncs) <= 1
        s = [s, symbstr(obj.Mat{i}, ['A', int2str(i)])]; %#ok<AGROW>
    else
        s = [s, '(', symbstr(obj.Mat{i}, ['A', int2str(i)]), ')']; %#ok<AGROW>
    end
    if i >= 2
        s = [s, strrep(Qistr, 'i', int2str(i-1))]; %#ok<AGROW>
    end
    if i < obj.N
        s = [s, ' + ']; %#ok<AGROW>
    end
end
fprintf('\n\t%s\n\n', s);

fprintf('with\n\n');

% Display coefficient values in non-SISO case
if size(obj, 1) > 1 || size(obj, 2) > 1
    for i=1:obj.N
        dispcoeff(obj.Mat{i}, ['A', int2str(i)]);
    end
end

fprintf('\tN:\t\t\t%d\n', obj.N);
fprintf('\tSize:\t\t%d-by-%d\n', size(obj, 1), size(obj, 2));
fprintf('\tParameters: %d (free: %d)\n', nparams(obj), nparams(obj, 'free'));
fprintf('\tNr. of scheduling signals:\t%d (extended: %d)\n', ...
    obj.SchedulingTimeMap.np, obj.SchedulingTimeMap.nrho);

fprintf('\n');

end

