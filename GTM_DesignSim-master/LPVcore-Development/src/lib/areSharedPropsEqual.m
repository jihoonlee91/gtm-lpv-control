function [v, ineq] = areSharedPropsEqual(obj1, obj2)
%ARESHAREDPROPSEQUAL Chech whether shared properties of
%objects are equal
%
%   Checks whether the shared public, non-hidden properties of two
%   classes are equal. Note: only properties of the same class are
%   considered. This function is used in the testing framework to ensure
%   that dynamic systems in LPVcore have properties that are consistent
%   with dynamic systems from MathWorks toolboxes (e.g. SS).
%
%   Syntax:
%       [v, ineq] = areSharedPropsEqual(obj1, obj2)
%
%   Example:
%       sys = lpvss(1, 1, 1, 1);
%       syslti = ss(1, 1, 1, 1);
%       assert(areSharedPropsEqual(sys, syslti))
%
%   Inputs:
%       obj1, obj2: any object
%
%   Outputs:
%       v: logical indicating whether the shared public, non-hidden properties of
%       obj1 and obj2 are equal.
%       ineq: cell array with names of properties that are inequal.
%

class1 = class(obj1);
class2 = class(obj2);

p1 = properties(class1);
p2 = properties(class2);

p = intersect(p1, p2);

v = true;
ineq = {};
for i=1:numel(p)
    p1 = obj1.(p{i});
    p2 = obj2.(p{i});
    if strcmp(class(p1), class(p2)) && ...
        ~isequal(obj1.(p{i}), obj2.(p{i}))
            v = false;
            ineq = [ineq(:)', p(i)];
    end
end

end