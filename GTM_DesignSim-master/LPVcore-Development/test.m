function test(~)
    % TEST Performs tests for LPVcore OR runs all example scripts.
    %
    %   This function is used by the CI/CD pipeline to automatically check
    %   each modification to LPVcore. You can invoke the test yourself by
    %   running this function from the root directory of LPVcore. Depending
    %   on whether an argument is specified, the function will run tests or
    %   example scripts to ensure there are no failures.
    %
    % Syntax:
    %
    %   test
    %
    %       Runs all tests in the "test" folder.
    %
    %   test("examples")
    %
    %       Runs all example scripts in the "examples" folder.
    %
    addpath(genpath('src'));

    if nargin == 0
        % Run unit tests
        fail = ~testdir('test');
    else
        % Run example scripts
        rundir('examples');
        fail = false;
    end
    
    % Close all generated plots
    close all;

    % Check failure
    if fail
        error('One or more tests have failed.');
    end
end