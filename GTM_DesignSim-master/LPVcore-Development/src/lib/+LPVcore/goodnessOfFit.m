function fit = goodnessOfFit(x, xref, cost_func)
%GOODNESSOFFIT Goodness of fit that has behavior of goodnessOfFit function
%in R2020a and later
%   Note: this function can be removed once support for R2019b is dropped.

% 9.7: R2019b
if verLessThan('matlab', '9.8') && ...
        any(strcmpi(cost_func, {'NRMSE', 'NMSE'}))
    fit = 1 - goodnessOfFit(x, xref, cost_func);
else
    fit = goodnessOfFit(x, xref, cost_func);
end

end

