%%
P = rss(3,2,2); 
K = rss(1,1,2); 
A = rss(1,1,1); 
W = rss(1,1,1); 
systemnames = 'W A K P'; 
input_to_P = '[W;A]'; 
input_to_W = '[deltemp]'; 
input_to_K = '[P(2)+noise;setpoint]'; 
input_to_A = '[K]'; 
inputvar = '[noise;deltemp;setpoint]'; 
outputvar = '[57.3*P(1);setpoint-P(2)]'; 
cleanupsysic = 'yes';
T = sysic
exist('inputvar') 

%%
P = rss(3,2,2); 
A = rss(1,1,1); 
W = rss(1,1,1); 
systemnames = 'W A P'; 
inputvar = '[noise;deltemp;setpoint;u]'; 
outputvar = '[57.3*P(1);setpoint-P(2);P(2)+noise;setpoint]'; 
input_to_W = '[deltemp]'; 
input_to_A = '[u]'; 
input_to_P = '[W;A]'; 
cleanupsysic = 'yes';
T = sysic
exist('inputvar') 

%%
GTM_lon = rss(4,2,1); 
systemnames = 'GTM_lon'; 
inputvar = '[noise;Pilot__lon;u]'; 
outputvar = '[57.3*GTM_lon(1);Pilot__lon-GTM_lon(2);GTM_lon(2)+noise;Pilot__lon]'; 
input_to_GTM_lon = '[u]'; 
cleanupsysic = 'yes';
T = sysic
exist('inputvar') 
