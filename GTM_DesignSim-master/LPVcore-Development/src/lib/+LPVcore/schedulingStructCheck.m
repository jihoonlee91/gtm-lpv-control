function ssorted = schedulingStructCheck(sys, s)
%SCHEDULINGSTRUCTCHECK Check structure array compatibility with system
%representation, and sorts fields of structure array to match system
%representation scheduling timemap

%
% Syntax:
%   ssorted = LPVcore.schedulingStructCheck(sys, s)
%
% Example:
%   Suppose sys has scheduling time map {'a', 'b'} and s has field names
%   {'b', 'a'}, then ssorted will have its field names permuted to match
%   {'a', 'b'}.

%% Input validation
narginchk(2, 2);
nargoutchk(0, 1);
assert(isa(sys, 'lpvrep'), '''sys'' must be lpvrep');
assert(isa(s, 'struct'), '''s'' must be struct array');

%% Perform check
% Sort field names to match scheduling time map
ssorted = orderfields(s);
if ~isequaln(fieldnames(ssorted), fieldnames(s)) && nargout == 0
    warning('Structure array passed as input is not sorted. However, no output arguments are used.');
end
f = fieldnames(ssorted);
assert(numel(f) == sys.Np, ...
    sprintf('Number of fields in structure array (%i) must match number of scheduling signals in system representation (%i)\n', ...
        numel(f), sys.Np));

% Check each element of struct is a scheduling signal
for i=1:numel(f)
    assert(strcmp(f{i}, sys.SchedulingTimeMap.Name{i}), ...
        sprintf('Field name ''%s'' is not a scheduling signal of system representation\n', f{i}));
end


end

