function f = SimulateDrive(motor, numMotors, lowGear, highGear, wheelDiameter,...
    robotResistance, Ev, Et, weight, Rt, dt, V0, targetDist, inputVoltage, ...
    currentLimit, voltageRamp)

results = DrivetrainSimulator(motor, numMotors, lowGear, highGear, wheelDiameter,...
    robotResistance, Ev, Et, weight, Rt, dt, V0, targetDist, inputVoltage, ...
    currentLimit, voltageRamp);

f = figure;

endTime = results(end,1);

% Pos vs Time
subplot(2,3,1)
plot(results(:,1), results(:,2)/12)
xlabel('Time (s)')
ylabel('Position (ft)')
grid on
xlim([0,endTime]);

% Vel vs Time
subplot(2,3,2)
plot(results(:,1), results(:,3)/12)
xlabel('Time (s)')
ylabel('Velocity (ft/s)')
grid on
xlim([0,endTime]);

% Accel vs Time
subplot(2,3,3)
plot(results(:,1), results(:,6)/12)
xlabel('Time (s)')
ylabel('Acceleration (ft/s^2)')
grid on
xlim([0,endTime]);

% Current vs Time
subplot(2,3,4)
plot(results(:,1), results(:,7))
xlabel('Time (s)')
ylabel('Current per Motor (A)')
grid on
xlim([0,endTime]);

% Voltage vs Time
subplot(2,3,5)
hold on
plot(results(:,1), results(:,10), 'DisplayName', 'System Voltage');
plot(results(:,1), results(:,8), 'DisplayName', 'Motor Voltage');
xlabel('Time (s)')
ylabel('Voltage (V)')
grid on
legend('Location', 'southeast')
xlim([0,endTime]);
hold off
end