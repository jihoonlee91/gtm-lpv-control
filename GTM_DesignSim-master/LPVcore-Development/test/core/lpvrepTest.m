function tests = lpvrepTest
%LPVREPTEST Test behavior of LPVREP
    tests = functiontests(localfunctions);
end

function setupOnce(~)
end

%% Test functions
function testExpandSignalNames_(testCase)
    signals = {'a', 'u(1)', 'u(2)', 'b', 'y(1)'};
    % Test
    test = struct(...
        'input', {{'y'},        {'a'}, {'u'},            {'u', 'b', 'a', 'y'}}, ...
        'output', {{'y(1)'},    {'a'}, {'u(1)', 'u(2)'}, {'u(1)', 'u(2)', 'b', 'a', 'y(1)'}});
    for i=1:numel(test)
        verifyEqual(testCase, ...
            lpvrep.expandSignalNames_(test(i).input, signals), test(i).output);
    end
end