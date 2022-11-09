clearvars; close all; clc; rng(1);
% Example script demonstrating the functionality of the grid-based LPV
% state-space object and global interpolation / approximation using the
% LPVINTERPSS function.
%
% The example script first creates a regular LPV
% state-space object using LPVSS, which is used to sample local linearizations. These
% local models together with the corresponding operating points are then
% used to create an LPVGRIDSS object.
%
% Then, the LPVGRIDSS object is converted into global form using several
% techniques available in the LPVINTERPSS function.
%

%% Configuration
% State space dimension. Note: for a grid-based SS representation, the
% state order must be constant for all grid points.
nx = 4;
% Input and output dimensions
ny = 1;
nu = 1;

%% Build LPV-SS model
% Define scheduling signals
p = preal('p', 'ct');
q = preal('q', 'ct');
% Define scheduling signal dependent matrix functions
A = -nx * eye(nx) + p * rand(nx) +      q * rand(nx);
B = randn(nx, nu) + p * randn(nx, nu) + q * randn(nx, nu);
C = randn(ny, nx) + p * randn(ny, nx) + q * randn(ny, nx);
D = randn(ny, nu) + p * randn(ny, nu) + q * randn(ny, nu);
% Create the LPVSS object
lpvsys = lpvss(A, B, C, D, 0, 'InputName', {'temperature'});

%% Sample from LPV-SS model
% Define a grid on the scheduling signals and sample
pGrid = linspace(-1, 0, 3);
qGrid = linspace(-1, 0, 3);
grid = struct('p', pGrid, 'q', qGrid);
sys = extractLocal(lpvsys, grid);

%% Store local linearization in LPVGRIDSS object
lpvgridsys = lpvgridss(sys, grid);
disp(lpvgridsys);

%% Method 1: Piece-wise linear interpolation
% The state-space coefficients are interpolated piecewise linearly. 
% Note that, since state-space coefficients are directly interpolated, it
% is important that all the local models are provided with coherent state
% bases.
lpvglobal_pwlinear = lpvinterpss(lpvgridsys, 'pwlinear');
lpvglobal_pwpoly   = lpvinterpss(lpvgridsys, 'pwpoly', 'spline');

%% Method 2: Global polynomial approximation
% The state-space coefficients are approximated by a global multivariable
% polynomial. Same as with piece-wise linear interpolation, the state bases
% must be coherent across the sampling grid.
%
% Global polynomial approximation is selected by setting the interpolation
% method argument to 'poly'. It requires another input: a structure array
% denoting the degree of the polynomial for each scheduling signal. Note
% that cross-terms (e.g., "p * q") are not supported currently.
%
% The state-space coefficients are approximated by a least-squares fit
% (i.e., based on "A \ b").
%
% Select degrees to be affine in 'p' and quadratic in 'q'.
degrees = struct('p', 1, 'q', 2);
lpvglobal_poly = lpvinterpss(lpvgridsys, 'poly', degrees);
% Note that the data-generating system is affine in both 'p' and 'q',
% indicating that the quadratic term in 'q' will be very small in
% magnitude. This can be checked by, e.g., running "lpvglobal_poly.D"

%% Method 3: Global behavioral approximation
% Behavioral approximation is supported in an H_infty sense. The worst-case
% H_infty error between the samples and the global model is minimized
% through a gradient-based optimization. In contrast to interpolation, this
% approximation scheme does NOT recover exactly the samples at the grid
% points. Furthermore, an initial global system representation must be supplied, from which the
% scheduling signal dependence and initial coefficient values are
% extracted.
init_sys = lpvsys;
% The global approximation is solved using HINFSTRUCT in the back-end,
% therefore the user may also specify an HINFSTRUCTOPTIONS object to
% configure aspects such as display and max. number of iterations.
hinfstructOpts = hinfstructOptions(...
    'Display', 'iter', ...
    'MaxIter', 50);
% Call LPVINTERPSS with interpolation method set to 'behavioral'
% In contrast to the methods based on interpolation of the state-space
% coefficients (such as 'pwlinear'), the state space bases of the local
% models need not be coherent.
lpvglobal_behavioral = lpvinterpss(lpvgridsys, 'behavioral', ...
    struct('init_sys', init_sys, 'hinfstructOptions', hinfstructOpts));

%% Validate results
% Results are validated by sampling the global models at a grid of points
% and comparing the samples with the true model.
pGridVal = linspace(-1, 0, 7);
qGridVal = linspace(-1, 0, 7);
for i=1:numel(pGridVal)
    for j=1:numel(qGridVal)
        % Sample models to compare
        sysTrue         = extractLocal(lpvsys,              [pGridVal(i), qGridVal(i)]);
        sysPwLinear     = extractLocal(lpvglobal_pwlinear,  [pGridVal(i), qGridVal(i)]);
        sysPwPoly       = extractLocal(lpvglobal_pwpoly,    [pGridVal(i), qGridVal(i)]);
        sysPoly         = extractLocal(lpvglobal_poly,      [pGridVal(i), qGridVal(i)]);
        sysBehavioral   = extractLocal(lpvglobal_behavioral,[pGridVal(i), qGridVal(i)]);
        % Compare by verifying Hinf error is sufficiently small
        assert(hinfnorm(sysTrue - sysPwLinear)    < 1E-10);
        assert(hinfnorm(sysTrue - sysPwPoly)      < 1E-10);
        assert(hinfnorm(sysTrue - sysPoly)        < 1E-10);
        assert(hinfnorm(sysTrue - sysBehavioral)  < 1E-10);
    end
end