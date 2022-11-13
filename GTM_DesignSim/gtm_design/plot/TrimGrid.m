load Trim.mat

alp_trim = squeeze(sch_alp_trim.Data.D);
throttle_trim = squeeze(sch_throttle_trim.Data.D);
elevator_trim = squeeze(sch_elevator_trim.Data.D);

DomainSize = size(sch_alp_trim.Domain);
%%
close all
figure
for i = 1:DomainSize(2) % altitude
for j = 1:DomainSize(1) % tas
    subplot(131), gca = plot([-25 -12.5 0],squeeze(alp_trim(j,i,:))); hold on
    xlabel('Span Variation [%]'), ylabel('Angle of Attack [deg]')
    if i == 1
        set(gca,'LineStyle','-')
    elseif i == 2
        set(gca,'LineStyle','--')
    elseif i == 3
        set(gca,'LineStyle',':')
    end
    if j == 1
        set(gca,'Color',[0.9290, 0.6940, 0.1250])
    elseif j == 2
        set(gca,'Color',[0.8500, 0.3250, 0.0980])
    elseif j == 3
        set(gca,'Color',[0.6350, 0.0780, 0.1840])
        if i == 3
            legend('0 ft, 70 knot','0 ft, 85 knot','0 ft, 100 knot','3000 ft, 70 knot','3000 ft, 85 knot','3000 ft, 100 knot','6000 ft, 70 knot','6000 ft, 85 knot','6000 ft, 100 knot')
        end
    end
    ylim([2 11]), grid on, grid minor

    subplot(132), gca = plot([-25 -12.5 0],squeeze(throttle_trim(i,j,:))); hold on
    xlabel('Span Variation [%]'), ylabel('Throttle Position [%]')
    if i == 1
        set(gca,'LineStyle','-')
    elseif i == 2
        set(gca,'LineStyle','--')
    elseif i == 3
        set(gca,'LineStyle',':')
    end
    if j == 1
        set(gca,'Color',[0.9290, 0.6940, 0.1250])
    elseif j == 2
        set(gca,'Color',[0.8500, 0.3250, 0.0980])
    elseif j == 3
        set(gca,'Color',[0.6350, 0.0780, 0.1840])
        if i == 3
            legend('0 ft, 70 knot','0 ft, 85 knot','0 ft, 100 knot','3000 ft, 70 knot','3000 ft, 85 knot','3000 ft, 100 knot','6000 ft, 70 knot','6000 ft, 85 knot','6000 ft, 100 knot')
        end
    end
    ylim([30 52]), grid on, grid minor
    
    subplot(133), gca = plot([-25 -12.5 0],squeeze(elevator_trim(i,j,:))); hold on
    xlabel('Span Variation [%]'), ylabel('Elevator Deflection [deg]')
    if i == 1
        set(gca,'LineStyle','-')
    elseif i == 2
        set(gca,'LineStyle','--')
    elseif i == 3
        set(gca,'LineStyle',':')
    end
    if j == 1
        set(gca,'Color',[0.9290, 0.6940, 0.1250])
    elseif j == 2
        set(gca,'Color',[0.8500, 0.3250, 0.0980])
    elseif j == 3
        set(gca,'Color',[0.6350, 0.0780, 0.1840])
        if i == 3
            legend('0 ft, 70 knot','0 ft, 85 knot','0 ft, 100 knot','3000 ft, 70 knot','3000 ft, 85 knot','3000 ft, 100 knot','6000 ft, 70 knot','6000 ft, 85 knot','6000 ft, 100 knot')
        end
    end
    ylim([-1.5 4.5]), grid on, grid minor
end
end

exportgraphics(gcf,'TrimGrid.pdf')
