function tf = isint(a)
%ISINT Returns true if input arguments represents scalar integer
%
%   This function differs from built-in ISINTEGER in that it also returns
%   true for rounded doubles.
%
%   Syntax:
%       tf = isint(a)
%

tf = isscalar(a) && isnumeric(a) && mod(a, 1) == 0;

end

