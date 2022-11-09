function r = lintdir(dirName, base)
%LINTDIR Recursively lints a directory.
%
%   Syntax:
%       r = lintdir(dirName, base)
%
%   Inputs:
%       dirName: name of the directory to lint (full or relative path).
%       base: base path to the directory.
%
%   Outputs:
%       r: boolean indicating whether linter errors were found.
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
            if ~lintdir(l(i).name, [base, '/', l(i).name])
                r = false;
            end
        elseif ~l(i).isdir && endsWith(l(i).name, '.m')
            c = checkcode(l(i).name);
            if numel(c) ~= 0
                r = false;
                for j=1:numel(c)
                    disp(['<a href="matlab: opentoline(''', pwd, '/', l(i).name, ''', ', ...
                        int2str(c(j).line), ')">', ...
                        int2str(c(j).line), '@', base, '/', l(i).name, ...
                        '</a>: ', c(j).message]);
                end
            end
        end
    end
    
    cd ..
end
