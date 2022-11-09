function sysc = lpvcore2lpvtools(sys, grid)
%LPVCORE2LPVTOOLS Convert LPVcore system representation to LPVTools format
%
% Converts an LPVCore system object to a PSS object from the LPVTools toolbox.
% Note that there are several limitations on the type of system that can be
% converted:
%
%   1. Only continuous-time systems are supported
%   2. Only system representations with static dependence on the scheduling variable are
%       supported.
%   3. Only state-space representations are supported (i.e., LPVLFR and
%   LPVSS are, but LPVIO is not).
%
% The conversion works by extracting local models at a rectangular grid of
% the scheduling signals. These local models are then stored in a state
% space array, together with the corresponding value of the scheduling signals.
%
% For more information, visit the website of LPVTools:
%
%   https://seiler.engin.umich.edu/software/
%
% Syntax:
%   sysc = lpvcore2lpvtools(sys, grid)
%
% Inputs:
%   sys: LPVcore system object (must be LPVLFR)
%   grid: cell array with as many elements as scheduling variables. 
%       Each element is a vector of sorted values that
%       specify the scheduling grid.
%
% Outputs:
%   sysc: converted LPVTools system object
%
% Example:
%   p = preal('p', 'ct')
%   sys = lpvss(-1-p, 1, 1, 1)
%   sysc = lpvcore2lpvtools(sys, { [-1, 1] })
%
% Example with 2 scheduling variables:
%   p = preal('p', 'ct')
%   q = preal('q', 'ct')
%   sys = lpvss(-2+p+q, 1, 1, 1)
%   sysc = lpvcore2lpvtools(sys, ...
%       { [-1, 1], [-2, 0] })
%
% See also LPVLFR

%% Input validation
assert(isa(sys, 'lpvlfr'), 'sys must be an LPVLFR object');
assert(isct(sys), 'sys must be a continuous-time system');
assert(hasStaticSchedulingDependence(sys), ...
    'sys must have static dependence on the scheduling variable');
assert(sys.Np > 0, 'there must be at least one scheduling signal');
assert(iscell(grid) && numel(grid) == sys.Np && ...
    all(cellfun(@(x) isnumeric(x) && isreal(x), grid)), ...
    ['grid must be a cell array with as many elements as ', ...
    'scheduling variables in sys. Each entry specifies the grid points ', ...
    'to include']);
%% Conversion
% Create the RGRID object (LPVTools)
IVName = sys.SchedulingTimeMap.Name;
IVData = grid;
IVRateBounds = cell2mat(sys.SchedulingTimeMap.RateBound);
r = rgrid(IVName, IVData, IVRateBounds);

% Iterate over points in domain and extract local state-space
% representation

% N is total number of operating points in the rectangular grid
N = prod(size(r)); %#ok<PSIZE> (numel is not properly implemented for RGRID)
p = cell(1, N);
for i=1:N
    p{i} = r(i).';  % Note the transpose: extractLocal expects row vector
end
sysLocal = extractLocal(sys, p);

% Convert cell array of local systems to regular array as expected by PSS
sysLocalArray(:, :, N) = sysLocal{N};
for i=1:N-1
    sysLocalArray(:, :, i) = sysLocal{i};
end
% Reshape local array (LVPTools expects as many dimensions as scheduling
% variables).
sysLocalArray = reshape(sysLocalArray, [size(r), 1]); %reshape requires a size array,
        %size(rgrid) is overloaded to return a scalar in case of single gridpoint

% Assemble LPVTools representation
sysc = pss(sysLocalArray, r);

end

