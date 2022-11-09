function BFR=bfr(y,y_hat)
% BFR Best Fit Rate in percentage
%
%   Syntax:
%       BFR = bfr(y, yhat)
%
%   Inputs:
%       y: reference output as a N-by-ny matrix, where ny is the number of
%           channels and N is the number of samples.
%       yhat: measured output of the same shape as y.
%
%   Outputs:
%       BFR: row vector with ny elements denoting the BFR for each channel.
%
BFR = NaN(1, size(y, 2));
for i=1:size(y, 2)
    BFR(i) = 100*max(1 - norm(y(:, i) - y_hat(:, i))/norm(y(:, i)-ones(size(y(:, i),1),1)*mean(y(:, i))),0);
end