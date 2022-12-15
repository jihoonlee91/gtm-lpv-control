set(0,'DefaultLineLineWidth',1)
tlim = 15;

%% auto_lon_step_tas_state
figure('units','centimeters','position',[0 6 20 20]);
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    [y,t] = step(CLlon5lpvu.Data(:,:,i,j,k),tlim);
    subplot(5,1,1), plot(t,y(:,1,1),'-b'), hold on
    subplot(5,1,2), plot(t,y(:,2,1),'-b'), hold on
    subplot(5,1,3), plot(t,y(:,3,1),'-b'), hold on
    subplot(5,1,4), plot(t,y(:,4,1),'-b'), hold on
    subplot(5,1,5), plot(t,y(:,5,1),'-b'), hold on

    [y,t] = step(CLlon5hinf.Data(:,:,i,j,k),tlim);
    subplot(5,1,1), plot(t,y(:,1,1),'--r')
    subplot(5,1,2), plot(t,y(:,2,1),'--r')
    subplot(5,1,3), plot(t,y(:,3,1),'--r')
    subplot(5,1,4), plot(t,y(:,4,1),'--r')
    subplot(5,1,5), plot(t,y(:,5,1),'--r')

    [y,t] = step(CLlon5hinf.Data(:,:,i,j,k),tlim);
    subplot(5,1,1), plot(t,y(:,1,1),':g')
    subplot(5,1,2), plot(t,y(:,2,1),':g')
    subplot(5,1,3), plot(t,y(:,3,1),':g')
    subplot(5,1,4), plot(t,y(:,4,1),':g')
    subplot(5,1,5), plot(t,y(:,5,1),':g')

    [y_r,t_r] = step(Wlon5.Wm(1,1),tlim);
    if isempty(t_r)
        t_r = linspace(0,tlim,numel(y_r)).';
        y_r = ones(numel(t_r),1);
    end
    subplot(5,1,1), plot(t_r,y_r*Wlon5.Wr(1,1)*Wlon5.Ps(1,1),'-.k')
    subplot(5,1,5), plot(t,y(:,1,1)*0,'-.k')
end
end
end

subplot(5,1,1), xlabel('Time [sec]'), ylabel('True airspeed [kt]'), legend('LPV','H_\infty','H_2','Model','orientation','horizontal','location','southeast'), grid on, grid minor, xlim tight, ylim padded
subplot(5,1,2), xlabel('Time [sec]'), ylabel('Angle of attack [deg]'), legend('LPV','H_\infty','H_2','orientation','horizontal','location','northeast'), grid on, grid minor, xlim tight, ylim padded
subplot(5,1,3), xlabel('Time [sec]'), ylabel('Pitch rate [deg/s]'), legend('LPV','H_\infty','H_2','orientation','horizontal','location','southeast'), grid on, grid minor, xlim tight, ylim padded
subplot(5,1,4), xlabel('Time [sec]'), ylabel('Pitch [deg]'), legend('LPV','H_\infty','H_2','orientation','horizontal','location','northeast'), grid on, grid minor, xlim tight, ylim padded
subplot(5,1,5), xlabel('Time [sec]'), ylabel('Altitude [ft]'), legend('LPV','H_\infty','H_2','Model','orientation','horizontal','location','northeast'), grid on, grid minor, xlim tight, ylim padded

exportgraphics(gcf,'figure/auto_lon_step_tas_state.png','resolution',300)

%% auto_lon_step_tas_input
figure('units','centimeters','position',[0 2 20 8]);
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    [y,t] = step(CLlon5lpvu.Data(:,:,i,j,k),tlim);
    subplot(2,1,1), plot(t,y(:,6,1),'-b'), hold on
    subplot(2,1,2), plot(t,y(:,7,1),'-b'), hold on

    [y,t] = step(CLlon5hinf.Data(:,:,i,j,k),tlim);
    subplot(2,1,1), plot(t,y(:,6,1),'--r')
    subplot(2,1,2), plot(t,y(:,7,1),'--r')
    
    [y,t] = step(CLlon5h2op.Data(:,:,i,j,k),tlim);
    subplot(2,1,1), plot(t,y(:,6,1),':g')
    subplot(2,1,2), plot(t,y(:,7,1),':g')
end
end
end

subplot(2,1,1), xlabel('Time [sec]'), ylabel('Throttle [%]'), legend('LPV','H_\infty','H_2','orientation','horizontal','location','northeast'), grid on, grid minor, xlim tight, ylim padded
subplot(2,1,2), xlabel('Time [sec]'), ylabel('Elevator [deg]'), legend('LPV','H_\infty','H_2','orientation','horizontal','location','northeast'), grid on, grid minor, xlim tight, ylim padded

exportgraphics(gcf,'figure/auto_lon_step_tas_input.png','resolution',300)

%% auto_lon_step_alt_state
figure('units','centimeters','position',[20 6 20 20]);
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    [y,t] = step(CLlon5lpvu.Data(:,:,i,j,k),tlim);
    subplot(5,1,1), plot(t,y(:,1,2),'-b'), hold on
    subplot(5,1,2), plot(t,y(:,2,2),'-b'), hold on
    subplot(5,1,3), plot(t,y(:,3,2),'-b'), hold on
    subplot(5,1,4), plot(t,y(:,4,2),'-b'), hold on
    subplot(5,1,5), plot(t,y(:,5,2),'-b'), hold on

    [y,t] = step(CLlon5hinf.Data(:,:,i,j,k),tlim);
    subplot(5,1,1), plot(t,y(:,1,2),'--r')
    subplot(5,1,2), plot(t,y(:,2,2),'--r')
    subplot(5,1,3), plot(t,y(:,3,2),'--r')
    subplot(5,1,4), plot(t,y(:,4,2),'--r')
    subplot(5,1,5), plot(t,y(:,5,2),'--r')
    
    [y,t] = step(CLlon5h2op.Data(:,:,i,j,k),tlim);
    subplot(5,1,1), plot(t,y(:,1,2),':g')
    subplot(5,1,2), plot(t,y(:,2,2),':g')
    subplot(5,1,3), plot(t,y(:,3,2),':g')
    subplot(5,1,4), plot(t,y(:,4,2),':g')
    subplot(5,1,5), plot(t,y(:,5,2),':g')
    
    [y_r,t_r] = step(Wlon5.Wm(2,2),tlim);
    if isempty(t_r)
        t_r = linspace(0,tlim,numel(y_r)).';
        y_r = ones(numel(t_r),1);
    end
    subplot(5,1,1), plot(t,y(:,1,1)*0,'-.k')    
    subplot(5,1,5), plot(t_r,y_r*Wlon5.Wr(2,2)*Wlon5.Ps(5,5),'-.k')
end
end
end

subplot(5,1,1), xlabel('Time [sec]'), ylabel('True airspeed [kt]'), legend('LPV','H_\infty','H_2','Model','orientation','horizontal','location','southeast'), grid on, grid minor, xlim tight, ylim padded
subplot(5,1,2), xlabel('Time [sec]'), ylabel('Angle of attack [deg]'), legend('LPV','H_\infty','H_2','orientation','horizontal','location','northeast'), grid on, grid minor, xlim tight, ylim padded
subplot(5,1,3), xlabel('Time [sec]'), ylabel('Pitch rate [deg/s]'), legend('LPV','H_\infty','H_2','orientation','horizontal','location','southeast'), grid on, grid minor, xlim tight, ylim padded
subplot(5,1,4), xlabel('Time [sec]'), ylabel('Pitch [deg]'), legend('LPV','H_\infty','H_2','orientation','horizontal','location','northeast'), grid on, grid minor, xlim tight, ylim padded
subplot(5,1,5), xlabel('Time [sec]'), ylabel('Altitude [ft]'), legend('LPV','H_\infty','H_2','Model','orientation','horizontal','location','southeast'), grid on, grid minor, xlim tight, ylim padded

exportgraphics(gcf,'figure/auto_lon_step_alt_state.png','resolution',300)

%% auto_lon_step_alt_input
figure('units','centimeters','position',[20 2 20 8]);
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    [y,t] = step(CLlon5lpvu.Data(:,:,i,j,k),tlim);
    subplot(2,1,1), plot(t,y(:,6,2),'-b'), hold on
    subplot(2,1,2), plot(t,y(:,7,2),'-b'), hold on

    [y,t] = step(CLlon5hinf.Data(:,:,i,j,k),tlim);
    subplot(2,1,1), plot(t,y(:,6,2),'--r')
    subplot(2,1,2), plot(t,y(:,7,2),'--r')

    [y,t] = step(CLlon5h2op.Data(:,:,i,j,k),tlim);
    subplot(2,1,1), plot(t,y(:,6,2),':g')
    subplot(2,1,2), plot(t,y(:,7,2),':g')
end
end
end

subplot(2,1,1), xlabel('Time [sec]'), ylabel('Throttle [%]'), legend('LPV','H_\infty','H_2','orientation','horizontal','location','northeast'), grid on, grid minor, xlim tight, ylim padded
subplot(2,1,2), xlabel('Time [sec]'), ylabel('Elevator [deg]'), legend('LPV','H_\infty','H_2','orientation','horizontal','location','southeast'), grid on, grid minor, xlim tight, ylim padded

exportgraphics(gcf,'figure/auto_lon_step_alt_input.png','resolution',300)
