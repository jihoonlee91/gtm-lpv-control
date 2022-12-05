set(0,'DefaultLineLineWidth',1)
tlim = 6;

%% auto_lon_step_tas_state
figure('units','centimeters','position',[0 6 20 16]);
for i = 1:num_sas
for j = 1:num_alt
for k = 1:num_mor
    [y,t] = step(CLlon2lpvu.Data(:,:,i,j,k),tlim);
    subplot(4,1,1), plot(t,y(:,1,1),'-b'), hold on
    subplot(4,1,2), plot(t,y(:,2,1),'-b'), hold on
    subplot(4,1,3), plot(t,y(:,3,1),'-b'), hold on
    subplot(4,1,4), plot(t,y(:,4,1),'-b'), hold on
    [y,t] = step(CLlon2hinf.Data(:,:,i,j,k),tlim);
    subplot(4,1,1), plot(t,y(:,1,1),'--r')
    subplot(4,1,2), plot(t,y(:,2,1),'--r')
    subplot(4,1,3), plot(t,y(:,3,1),'--r')
    [y_r,t_r] = step(Wlon2.Wm(1,1),tlim);
    if isempty(t_r)
        t_r = linspace(0,tlim,numel(y_r)).';
        y_r = ones(numel(t_r),1);
    end
    plot(t_r,-y_r*Wlon2.Wr(1,1)*Wlon2.Ps(3,3),'-.k')
    subplot(4,1,4), plot(t,y(:,4,1),'--r')
end
end
end

subplot(4,1,1), xlabel('Time [sec]'), ylabel('Angle of attack [deg]'), legend('LPV','H_\infty','location','northeast'), grid on, grid minor, xlim tight, ylim padded
subplot(4,1,2), xlabel('Time [sec]'), ylabel('Pitch rate [deg/s]'), legend('LPV','H_\infty','location','northeast'), grid on, grid minor, xlim tight, ylim padded
subplot(4,1,3), xlabel('Time [sec]'), ylabel('Normal acceleration [kt]'), legend('LPV','H_\infty','Model','location','southeast'), grid on, grid minor, xlim tight, ylim padded
subplot(4,1,4), xlabel('Time [sec]'), ylabel('Elevator [deg]'), legend('LPV','H_\infty','location','southeast'), grid on, grid minor, xlim tight, ylim padded

exportgraphics(gcf,'figure/auto_lon_step_tas_state.png','resolution',300)