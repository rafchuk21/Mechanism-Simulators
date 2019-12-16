motor = 'MiniCIM';
numMotors = 6;
wheelDiameter = 4;
robotResistance = .02;
velEfficiency = 0.9;
torqueEfficiency = 0.9;
weight = 154;
resistTorque = 7;
dt = .001;
v0 = 0;
targetDist = 240;
inputVoltage = 12;
currentLimit = 60;
voltageRamp = 120;

minGearing = 2;
maxGearing = 20;
delta = .1;

motordata = MotorData();
freeSpeed = motordata.(motor)(1);

times = NaN(length(minGearing:delta:maxGearing));

theoreticalSpeeds = freeSpeed/60*pi*wheelDiameter*velEfficiency./[minGearing:delta:maxGearing]/12;

for lowGear = minGearing:delta:maxGearing
    for highGear = minGearing:delta:lowGear
        disp(['Low Gear: ', num2str(lowGear), ' High Gear: ', num2str(highGear)])
        results = DrivetrainSimulator(motor, numMotors, lowGear, highGear, ...
            wheelDiameter, robotResistance, velEfficiency, torqueEfficiency,...
            weight, resistTorque, dt, v0, targetDist, inputVoltage, ...
            currentLimit, voltageRamp);
        times(round((lowGear-minGearing)/delta + 1), round((highGear-minGearing)/delta+1))...
            = max(results(:,1));
    end
end

times(times > 10) = NaN;

f = figure;
f.Position = [100,100,800,800];
hold on
s = surface(minGearing:delta:maxGearing, minGearing:delta:maxGearing, times', 'EdgeColor', 'none');
xlabel('Low Gear (X:1)')
ylabel('High Gear (X:1)')

[M, I] = min(times);
highGearIdx = find(M == min(M), 1);
highGear = minGearing + delta * (highGearIdx - 1);
lowGearIdx = I(highGearIdx);
lowGear = minGearing + delta * (lowGearIdx - 1);
minTime = times(lowGearIdx, highGearIdx);
p1 = plot3([0.0,maxGearing],[0.0,maxGearing/2.16],[10,10], '--', 'DisplayName', '2.16x Spread');
p2 = plot3([0.0,maxGearing],[0.0,maxGearing/2.65],[10,10], '--', 'DisplayName', '2.65x Spread');
p3 = plot3([0.0,maxGearing],[0.0,maxGearing/3.68],[10,10], '--', 'DisplayName', '3.68x Spread');
scatter3(lowGear, highGear, minTime+.05, 'filled');
legend([p1, p2, p3], 'Location', 'northwest')
text(lowGear, highGear, 10, ['(', num2str(lowGear), ', ', num2str(highGear), '): ', num2str(minTime), ' s']);
xlim([minGearing, maxGearing]);
ylim([minGearing, maxGearing]);
set(gca, 'ColorScale', 'log');
cb = colorbar('southoutside');
t = title(cb, ['Time to target: ', num2str(targetDist), ' in']);
t.Position = [232.5,-30,0];
t = title({'Time to Target vs Gearings'; [num2str(numMotors), ' ', motor, ', ', ...
    num2str(wheelDiameter), 'in Wheel, ' num2str(currentLimit), 'A Current Limit']});
t.Position = [t.Position(1), -3.5, t.Position(3)];
ax1 = gca;
ax2 = axes('Position', ax1.Position,...
    'XAxisLocation','top',...
    'YAxisLocation','right',...
    'Color','none');
xlim(ax2, [minGearing, maxGearing]);
ylim(ax2, [minGearing, maxGearing]);
xticks(ax2, ax1.XTick);
yticks(ax2, ax1.YTick);
%xticks(ax2, sort([ax1.XTick, lowGear]));
%yticks(ax2, sort([ax1.YTick, highGear]));
xticklabels(ax2, sprintfc('%.2f', freeSpeed/60*pi*wheelDiameter*velEfficiency/12./ax2.XTick));
yticklabels(ax2, sprintfc('%.2f', freeSpeed/60*pi*wheelDiameter*velEfficiency/12./ax2.YTick));
% set(ax2, 'xdir', 'reverse');
% set(ax2, 'ydir', 'reverse');
xlabel('Theoretical Free Speed (ft/s)')
ylabel('Theoretical Free Speed (ft/s)')
grid on
hold off

SimulateDrive(motor, numMotors, lowGear, highGear, ...
            wheelDiameter, robotResistance, velEfficiency, torqueEfficiency,...
            weight, resistTorque, dt, v0, targetDist, inputVoltage, ...
            currentLimit, voltageRamp );