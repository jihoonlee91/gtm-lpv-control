function r = testdir(dirName, base)
%TESTDIR Recursively runs test files in a directory.
%
%   Syntax:
%       r = testdir(dirName, base)
%
%   Inputs:
%       dirName: name of the directory to test (full or relative path).
%       base: base path to the directory.
%
%   Outputs:
%       r: boolean indicating whether test errors were found.
%

    if nargin <= 1
        base = dirName;
    end

    r = true;
    cd(dirName);
    
    l = dir;
    for i=1:numel(l)
        if l(i).isdir && ~strcmp(l(i).name(1), '.')
            % recurse
            if ~testdir(l(i).name, [base, '/', l(i).name])
                r = false;
            end
        elseif ~l(i).isdir && endsWith(l(i).name, '.m')
            c = runtests(l(i).name);
            r = processResults(c, r);
        end
    end
    
    cd ..
end

%% Process results and return overall failure status.
function r = processResults(results, r)
    numFailed = sum([results.Failed]);
    if numFailed > 0
        disp(results)
    end
    if ~r || numFailed > 0
        r = false;
    else
        r = true;
    end
end
