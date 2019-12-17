function f = SimulateDrive(motor, numMotors, lowGear, highGear, wheelDiameter,...
    robotResistance, Ev, Et, weight, CoF, Rt, dt, V0, targetDist, inputVoltage, ...
    currentLimit, voltageRamp)
% Runs the DrivetrainSimulator function and creates plots.

results = DrivetrainSimulator(motor, numMotors, lowGear, highGear, wheelDiameter,...
    robotResistance, Ev, Et, weight, CoF, Rt, dt, V0, targetDist, inputVoltage, ...
    currentLimit, voltageRamp);

time = results.time;
pos = results.position;
vel = results.velocity;
accel = results.acceleration;
current = results.current;
voltage = results.voltage;
desVoltage = results.desiredVoltage;
sysVoltage = results.systemVoltage;

motorData = MotorData();
freeSpeed = motorData.(motor)(1);

f = figure;

endTime = time(end);

% Pos vs Time
subplot(2,3,1)
plot(time, pos/12)
xlabel('Time (s)')
ylabel('Position (ft)')
grid on
xlim([0,endTime]);

% Vel vs Time
subplot(2,3,2)
plot(time, vel/12)
xlabel('Time (s)')
ylabel('Velocity (ft/s)')
grid on
xlim([0,endTime]);

% Accel vs Time
subplot(2,3,3)
plot(time, accel/12)
xlabel('Time (s)')
ylabel('Acceleration (ft/s^2)')
grid on
xlim([0,endTime]);

% Current vs Time
subplot(2,3,4)
plot(time, current)
xlabel('Time (s)')
ylabel('Current per Motor (A)')
grid on
xlim([0,endTime]);

% Voltage vs Time
subplot(2,3,5)
hold on
p1 = plot(time, sysVoltage, 'DisplayName', 'System Voltage');
p2 = plot(time, voltage, 'DisplayName', 'Motor Voltage');
plot([0.0,endTime],[7.0,7.0], '--r');
xlabel('Time (s)')
ylabel('Voltage (V)')
grid on
legend([p1, p2], 'Location', 'southeast')
xlim([0,endTime]);
ylim([0,inputVoltage]);
hold off

s = subplot(2,3,6);
g2s = freeSpeed/60*pi*wheelDiameter*Ev/inputVoltage;
annotation(f, 'textbox', [s.Position(1),s.Position(2), s.Position(3)*1.1,s.Position(4)*.9], 'string',...
    {sprintf('Gearing (X:1): %.1f/%.1f', lowGear, highGear),...
    sprintf('Speeds (ft/s): %.1f/%.1f', g2s/lowGear, g2s/highGear),...
    sprintf('Time to Target: %.2f s', endTime),...
    sprintf('Power Use: %.2g A*h',numMotors*sum(current)*dt/3600),...
    sprintf('Current Limit: %.0f A', currentLimit), ...
    sprintf('Voltage Ramp: %.0f V/s', voltageRamp),...
    sprintf('CoF: %.1f', CoF)});
delete(s);    

sgtitle(f, {'Time to Target vs Gearings'; [num2str(numMotors), ' ', motor, ', ', ...
    num2str(wheelDiameter), 'in Wheel, ' num2str(inputVoltage), 'V Input']});
f.Position(3) = f.Position(3) * 1.2;
end