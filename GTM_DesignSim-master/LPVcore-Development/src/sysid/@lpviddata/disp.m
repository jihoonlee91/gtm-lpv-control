function disp(obj)
% DISP Provide info on the lpviddata object

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    fprintf('\nTime domain data set with %d samples.\n',obj.lengthDataSet);
    fprintf('Sample time: %g %s.\n\n',obj.Ts,obj.TimeUnit);
    

    space = 3;
    offsO = cellfun('length',obj.OutputName(:));
    offsS = cellfun('length',obj.SchedulingName(:));
    offsI = cellfun('length',obj.InputName(:));

    offs  = max([offsO;offsS;offsI;7]);

    tabL = offs+2*space;

    if ~isempty(obj.OutputData)
        
        fprintf('Outputs%sUnit (if specified)\n',repmat(' ',[tabL-7 1]));
        for i = 1:length(obj.OutputName)
            fprintf('%s%s',repmat(' ',[space 1]),obj.OutputName{i});
            
            if ~isempty(obj.OutputUnit)
                fprintf('%s%s\n',repmat(' ',[tabL-offsO(i) 1]),obj.OutputUnit{i});
            else
                fprintf('\n');
            end
        end
        fprintf('\n');
    end
    
    
    if ~isempty(obj.SchedulingData)
        fprintf('Scheduling%sUnit (if specified)\n',repmat(' ',[tabL-10 1]));
        for i = 1:length(obj.SchedulingName)
            fprintf('%s%s',repmat(' ',[space 1]),obj.SchedulingName{i});

            if ~isempty(obj.SchedulingUnit)
                fprintf('%s%s\n',repmat(' ',[tabL-offsS(i) 1]),obj.SchedulingUnit{i});
            else
                fprintf('\n');
            end
        end
        fprintf('\n');
    end

    if ~isempty(obj.InputData)
        
        fprintf('Inputs%sUnit (if specified)\n',repmat(' ',[tabL-6 1]));
        for i = 1:length(obj.InputName)
            fprintf('%s%s',repmat(' ',[space 1]),obj.InputName{i});
            
            if ~isempty(obj.InputUnit)
                fprintf('%s%s\n',repmat(' ',[tabL-offsI(i) 1]),obj.InputUnit{i});
            else
                fprintf('\n');
            end
        end
        fprintf('\n');
    end
    
end