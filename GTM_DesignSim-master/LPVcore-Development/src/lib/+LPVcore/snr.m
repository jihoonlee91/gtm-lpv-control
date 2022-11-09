function r = snr(x, y)
%SNR Signal-to-Noise ratio
%
%   Syntax:
%       r = snr(x, y)
%
%   Inputs:
%       x: real-valued input signal
%       y: noise estimate
%
%   Outputs:
%       r: signal-to-noise ratio in dB
%
%   See also: SNR from Signal Processing Toolbox

% SNR = 10 * log10( E[x^2] / E[y^2] )
%     = 20 * log10( norm(x) / norm(y) )

r = 20 * log10( norm(x) / norm(y) );

end

