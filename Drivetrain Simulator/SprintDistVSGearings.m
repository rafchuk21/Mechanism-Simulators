% Optimizes low gear and high gear for a given robot and sprint distance.

motor = 'NEO';
numMotors = 6;
wheelDiameter = 4;
robotResistance = .02;
velEfficiency = 0.9;
torqueEfficiency = 0.9;
weight = 154;
CoF = 1.1;
resistTorque = 7;
dt = .001;
v0 = 0;
targetDist = 27*12;
inputVoltage = 12;
currentLimit = 40;
voltageRamp = 9999;

% Range of gearings (X:1 to Y:1) and increment of gearings
minGearing = 2;
maxGearing = 15;
delta = .2;

% Pull motor data info
motordata = MotorData();
freeSpeed = motordata.(motor)(1);

% Initialize array to store simulation times
times = NaN(length(minGearing:delta:maxGearing));

% Compute theoretical speeds
theoreticalSpeeds = freeSpeed/60*pi*wheelDiameter*velEfficiency./[minGearing:delta:maxGearing]/12;

% For each value of the low gear
for lowGear = minGearing:delta:maxGearing
    % For each value of the high gear
    for highGear = minGearing:delta:lowGear
        % Simulate the robot w/ that low gear and high gear combination
        disp(['Low Gear: ', num2str(lowGear), ' High Gear: ', num2str(highGear)])
        results = DrivetrainSimulator(motor, numMotors, lowGear, highGear, ...
            wheelDiameter, robotResistance, velEfficiency, torqueEfficiency,...
            weight, CoF, resistTorque, dt, v0, inputVoltage, ...
            currentLimit, voltageRamp);
        % Get and store the simulated times
        t = results.time;
        times(round((lowGear-minGearing)/delta + 1), round((highGear-minGearing)/delta+1))...
            = max(t);
    end
end

% Eliminate anything that takes too long
times(times > 10) = NaN;

% Create figure for plot of time vs high gear vs low gear.
f = figure;
f.Position = [100,100,800,800];
hold on
s = surface(minGearing:delta:maxGearing, minGearing:delta:maxGearing, times', 'EdgeColor', 'none');
xlabel('Low Gear (X:1)')
ylabel('High Gear (X:1)')

% Get value, idx for the fastest time
[M, I] = min(times);
% Get high gear, low gear info for the fastest time
highGearIdx = find(M == min(M), 1);
highGear = minGearing + delta * (highGearIdx - 1);
lowGearIdx = I(highGearIdx);
lowGear = minGearing + delta * (lowGearIdx - 1);
minTime = times(lowGearIdx, highGearIdx);
% Create lines on the 3d plot for the three shifter spreads on VexPro 3CIM
% Ball Shifter
p1 = plot3([0.0,maxGearing],[0.0,maxGearing/2.16],[10,10], '--', 'DisplayName', '2.16x Spread');
p2 = plot3([0.0,maxGearing],[0.0,maxGearing/2.65],[10,10], '--', 'DisplayName', '2.65x Spread');
p3 = plot3([0.0,maxGearing],[0.0,maxGearing/3.68],[10,10], '--', 'DisplayName', '3.68x Spread');
% Put a point for the optimal low gear/high gear combination (+.05 to time
% is just so the point shows up above the plot surface)
scatter3(lowGear, highGear, minTime+.05, 'filled');
legend([p1, p2, p3], 'Location', 'northwest')
text(lowGear, highGear, 10, ['(', num2str(lowGear), ', ', num2str(highGear), '): ', num2str(minTime), ' s']);
xlim([minGearing, maxGearing]);
ylim([minGearing, maxGearing]);
% Create colorbar for the times
set(gca, 'ColorScale', 'log');
cb = colorbar('southoutside');
t = title(cb, ['Time to target: ', num2str(targetDist), ' in']);
t.Position = [232.5,-30,0];
t = title({'Time to Target vs Gearings'; [num2str(numMotors), ' ', motor, ', ', ...
    num2str(wheelDiameter), 'in Wheel, ' num2str(currentLimit), 'A Current Limit']});
t.Position = [t.Position(1), -3.5, t.Position(3)];
ax1 = gca;
% Create axes for showing free speed values
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
axes(ax1);
grid on
hold off

% Create plot for optimal 2 speed drive
f2 = SimulateDrive(motor, numMotors, lowGear, highGear, ...
            wheelDiameter, robotResistance, velEfficiency, torqueEfficiency,...
            weight, CoF, resistTorque, dt, v0, inputVoltage, ...
            currentLimit, voltageRamp);
        
% For values where low gear = high gear (i.e. single speed drives) find the
% best one
sstimes = diag(times);
[~, ssidx] = min(sstimes);
ssgear = minGearing + delta * (ssidx - 1);

% Create plot for optimal single speed drive
f3 = SimulateDrive(motor, numMotors, ssgear, ssgear, ...
            wheelDiameter, robotResistance, velEfficiency, torqueEfficiency,...
            weight, CoF, resistTorque, dt, v0, inputVoltage, ...
            currentLimit, voltageRamp);
