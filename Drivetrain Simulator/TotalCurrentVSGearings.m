motor = 'Falcon';
numMotors = 6;
wheelDiameter = 4;
robotResistance = .02;
velEfficiency = 0.9;
torqueEfficiency = 0.9;
weight = 154;
resistTorque = 7;
dt = .001;
v0 = 0;
targetDist = 480;
inputVoltage = 12;

minGearing = 1;
maxGearing = 15;
delta = .1;

powers = NaN(length(minGearing:delta:maxGearing));

for lowGear = minGearing:delta:maxGearing
    for highGear = minGearing:delta:lowGear
        disp(['Low Gear: ', num2str(lowGear), ' High Gear: ', num2str(highGear)])
        results = DrivetrainSimulator(motor, numMotors, lowGear, highGear, ...
            wheelDiameter, robotResistance, velEfficiency, torqueEfficiency,...
            weight, resistTorque, dt, v0, targetDist, inputVoltage);
        powers(round((lowGear-minGearing)/delta + 1), round((highGear-minGearing)/delta+1))...
            = sum(results(:,7))*dt;
    end
end

figure
hold on
s = surface(minGearing:delta:maxGearing, minGearing:delta:maxGearing, powers', 'EdgeColor', 'none');
cb = colorbar;
title(cb, ['Power Use [A*s] to Destination: ', num2str(targetDist), ' in'])
xlabel('Low Gear (X:1)')
ylabel('High Gear (X:1)')

[M, I] = min(powers);
highGearIdx = find(M == min(M), 1);
highGear = minGearing + delta * (highGearIdx - 1);
lowGearIdx = I(highGearIdx);
lowGear = minGearing + delta * (lowGearIdx - 1);
minPower = powers(lowGearIdx, highGearIdx);
scatter(lowGear, highGear, minPower);
text(lowGear, highGear, minPower+.1, ['(', num2str(lowGear), ', ', num2str(highGear), '): ', num2str(minPower), ' A*s']);
hold off
    