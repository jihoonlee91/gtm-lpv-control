function h0 = plot(obj)
%PLOT Plot the data in the data set (lpviddata object).
%
%   plot(data) will display a figure of the measurements recoreded in the
%   data objects.
%
%   h = plot(obj) returns the function handle in h.
%
%   See also LPVIDDATA

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    ny = size(obj.y,2);
    np = size(obj.p,2);
    nu = size(obj.u,2);
    t  = obj.t;
    
    if ny == np && np == nu && ny == nu
        n = ny;
        z = 3;
    elseif (ny == 0 || nu == 0) && (np == nu || np == ny)
        n = np;
        z = 2;
    else
        if ny == 0; n = np*nu; z = 2;
        elseif nu == 0; n = ny*np; z = 2;
        else; n = ny*np*nu; z = 3; 
        end
    end

    figure; clf;
    
    if ny ~= 0
       for i = 1:ny
           shft = n/ny;
           subplot(z,n,shft*(i-1)+1 : shft*i);
           plot(t,obj.OutputData(:,i));
           title(obj.OutputName{i});
       end
       off = n; off1 = n*2;
    else
        off = 0; off1 = n;
    end
    
    for i = 1:np
        shft = n/np;
        subplot(z,n,shft*(i-1)+1+off : shft*i+off);
        plot(t,obj.SchedulingData(:,i));
        title(obj.SchedulingName{i});
    end
    
    
    if nu ~= 0
        for i = 1:nu
            shft = n/nu;
            subplot(z,n,shft*(i-1)+1+off1 : shft*i+off1);
            plot(t,obj.InputData(:,i));
            title(obj.InputName{i});
        end
    end
    
    a = axes;
    t1 = title('Input-Scheduling-Output Data');
    y1 = ylabel('Amplitude');
    x1 = xlabel(['Time (',obj.TimeUnit,')']);
    a.Visible = 'off';
    t1.Visible = 'on';
    set(t1,'Position',[0.5000    1.0400    0.5000])
    y1.Visible = 'on';
    x1.Visible = 'on';
    drawnow;
    
    if nargout
        h0 = gcf;
    end

end

