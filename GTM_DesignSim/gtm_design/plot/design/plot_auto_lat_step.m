set(0,'DefaultLineLineWidth',1)
tlim = 4;

%% auto_lat_step_bet_state
figure('units','centimeters','position',[0 6 13 20]);
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    [y,t] = step(CLlat4lpvu.Data(:,:,i,j,k),tlim);
    subplot(4,1,1), plot(t,y(:,1,1),'-b'), hold on
    subplot(4,1,2), plot(t,y(:,2,1),'-b'), hold on
    subplot(4,1,3), plot(t,y(:,3,1),'-b'), hold on
    subplot(4,1,4), plot(t,y(:,4,1),'-b'), hold on
    [y,t] = step(CLlat4hinf.Data(:,:,i,j,k),tlim);
    subplot(4,1,1), plot(t,y(:,1,1),'--r')
    [y_r,t_r] = step(Wlat4.Wm(1,1),tlim);
    if isempty(t_r)
        t_r = linspace(0,tlim,numel(y_r)).';
        y_r = ones(numel(t_r),1);
    end
    plot(t_r,y_r*Wlat4.Wr(1,1)*Wlat4.Ps(1,1),'-.k')
    subplot(4,1,2), plot(t,y(:,2,1),'--r')
    subplot(4,1,3), plot(t,y(:,3,1),'--r')
    subplot(4,1,4), plot(t,y(:,4,1),'--r')
    plot(t,y(:,1,1)*0,'-.k')
end
end
end

subplot(4,1,1), xlabel('Time [sec]'), ylabel('Sideslip [deg]'), legend('LPV','H_\infty','Model','location','best'), grid on, grid minor, xlim tight, ylim padded
subplot(4,1,2), xlabel('Time [sec]'), ylabel('Roll rate [deg/s]'), legend('LPV','H_\infty','location','best'), grid on, grid minor, xlim tight, ylim padded
subplot(4,1,3), xlabel('Time [sec]'), ylabel('Yaw rate [deg/s]'), legend('LPV','H_\infty','location','best'), grid on, grid minor, xlim tight, ylim padded
subplot(4,1,4), xlabel('Time [sec]'), ylabel('Roll [deg]'), legend('LPV','H_\infty','Model','location','best'), grid on, grid minor, xlim tight, ylim padded

exportgraphics(gcf,'figure/auto_lat_step_bet_state.png','resolution',300)

%% auto_lat_step_bet_input
figure('units','centimeters','position',[13 6 13 20]);
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    [y,t] = step(CLlat4lpvu.Data(:,:,i,j,k),tlim);
    subplot(3,1,1), plot(t,y(:,5,1),'-b'), hold on
    subplot(3,1,2), plot(t,y(:,6,1),'-b'), hold on
    subplot(3,1,3), plot(t,y(:,7,1),'-b'), hold on
    [y,t] = step(CLlat4hinf.Data(:,:,i,j,k),tlim);
    subplot(3,1,1), plot(t,y(:,5,1),'--r')
    subplot(3,1,2), plot(t,y(:,6,1),'--r')
    subplot(3,1,3), plot(t,y(:,7,1),'--r')
end
end
end

subplot(3,1,1), xlabel('Time [sec]'), ylabel('Aileron [%]'), legend('LPV','H_\infty','location','best'), grid on, grid minor, xlim tight, ylim padded
subplot(3,1,2), xlabel('Time [sec]'), ylabel('Rudder [deg]'), legend('LPV','H_\infty','location','best'), grid on, grid minor, xlim tight, ylim padded
subplot(3,1,3), xlabel('Time [sec]'), ylabel('Asy. morphing [deg]'), legend('LPV','H_\infty','location','best'), grid on, grid minor, xlim tight, ylim padded

exportgraphics(gcf,'figure/auto_lat_step_bet_input.png','resolution',300)

%% auto_lat_step_phi_state
figure('units','centimeters','position',[26 6 13 20]);
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    [y,t] = step(CLlat4lpvu.Data(:,:,i,j,k),tlim);
    subplot(4,1,1), plot(t,y(:,1,2),'-b'), hold on
    subplot(4,1,2), plot(t,y(:,2,2),'-b'), hold on
    subplot(4,1,3), plot(t,y(:,3,2),'-b'), hold on
    subplot(4,1,4), plot(t,y(:,4,2),'-b'), hold on
    [y,t] = step(CLlat4hinf.Data(:,:,i,j,k),tlim);
    subplot(4,1,1), plot(t,y(:,1,2),'--r')
    plot(t,y(:,1,1)*0,'-.k')    
    subplot(4,1,2), plot(t,y(:,2,2),'--r')
    subplot(4,1,3), plot(t,y(:,3,2),'--r')
    subplot(4,1,4), plot(t,y(:,4,2),'--r')
    [y_r,t_r] = step(Wlat4.Wm(2,2),tlim);
    if isempty(t_r)
        t_r = linspace(0,tlim,numel(y_r)).';
        y_r = ones(numel(t_r),1);
    end
    plot(t_r,y_r*Wlat4.Wr(2,2)*Wlat4.Ps(4,4),'-.k')
end
end
end

subplot(4,1,1), xlabel('Time [sec]'), ylabel('Sideslip [deg]'), legend('LPV','H_\infty','Model','location','best'), grid on, grid minor, xlim tight, ylim padded
subplot(4,1,2), xlabel('Time [sec]'), ylabel('Roll rate [deg/s]'), legend('LPV','H_\infty','location','best'), grid on, grid minor, xlim tight, ylim padded
subplot(4,1,3), xlabel('Time [sec]'), ylabel('Yaw rate [deg/s]'), legend('LPV','H_\infty','location','best'), grid on, grid minor, xlim tight, ylim padded
subplot(4,1,4), xlabel('Time [sec]'), ylabel('Roll [deg]'), legend('LPV','H_\infty','Model','location','best'), grid on, grid minor, xlim tight, ylim padded

exportgraphics(gcf,'figure/auto_lat_step_phi_state.png','resolution',300)

%% auto_lat_step_phi_input
figure('units','centimeters','position',[39 6 13 20]);
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    [y,t] = step(CLlat4lpvu.Data(:,:,i,j,k),tlim);
    subplot(3,1,1), plot(t,y(:,5,2),'-b'), hold on
    subplot(3,1,2), plot(t,y(:,6,2),'-b'), hold on
    subplot(3,1,3), plot(t,y(:,7,2),'-b'), hold on
    [y,t] = step(CLlat4hinf.Data(:,:,i,j,k),tlim);
    subplot(3,1,1), plot(t,y(:,5,2),'--r')
    subplot(3,1,2), plot(t,y(:,6,2),'--r')
    subplot(3,1,3), plot(t,y(:,7,2),'--r')
end
end
end

subplot(3,1,1), xlabel('Time [sec]'), ylabel('Aileron [%]'), legend('LPV','H_\infty','location','best'), grid on, grid minor, xlim tight, ylim padded
subplot(3,1,2), xlabel('Time [sec]'), ylabel('Rudder [deg]'), legend('LPV','H_\infty','location','best'), grid on, grid minor, xlim tight, ylim padded
subplot(3,1,3), xlabel('Time [sec]'), ylabel('Asy. morphing [deg]'), legend('LPV','H_\infty','location','best'), grid on, grid minor, xlim tight, ylim padded

exportgraphics(gcf,'figure/auto_lat_step_phi_input.png','resolution',300)
