function SimulationResults = ...
    DrivetrainSimulator(motor, numMotors, lowGear, highGear, wheelDiameter,...
    robotResistance, Ev, Et, weight, CoF, Rt, dt, V0, targetDist, inputVoltage, ...
    currentLimit, voltageRamp)
% Simulates the behavior of a drivetrain given a "full ahead" voltage.
% Accounts for voltage sag and implements current limiting and voltage
% ramping. Does not come to rest at target distance and does not account
% for wheel slip.
%
% motor: String name of motor on drivetrain
% numMotors: Number of motors on drivetrain
% lowGear: Low gearing of drivetrain (X:1)
% highGear: High gearing of drivetrain(X:1)
% wheelDiameter: Diameter of wheel (in)
% robotResistance: Resistance in electronics of robot including battery
%                  internal resistance (ohm)
% Ev: Velocity efficiency as a fraction from 0 to 1
% Et: Torque efficiency as a fraction from 0 to 1
% weight: Robot weight (lbs)
% CoF: Wheel coefficient of friction
% Rt: Wheel resistance torque
% dt: Time interval for Euler's method (s)
% V0: Initial velocity (in/s)
% targetDist: Sprint distance to end simulation (in)
% inputVoltage: Constant voltage input, available battery voltage (V)
% currentLimit: Current limit (A)
% voltageRamp: Voltage ramp (V/s)

nmToInLb = 8.85074579;
gsToInchPerSecSquared = 386.09;

radius = wheelDiameter/2;

motorData = MotorData();
freeSpeed = motorData.(motor)(1);
stallTorque = motorData.(motor)(2);
stallCurrent = motorData.(motor)(3);
freeCurrent = motorData.(motor)(4);

Vin = 0.0;

motorForce = stallTorque*nmToInLb*numMotors*Et/(radius)*[lowGear,highGear];
motorResistance = 12/stallCurrent;
voltage = Vin/(1+numMotors*(robotResistance/motorResistance));
twelveVoltSpeed = freeSpeed/60*pi*wheelDiameter*Ev*[1/lowGear, 1/highGear];
kV = 12./twelveVoltSpeed;
kC = 12*Rt*radius./motorForce;
maxAccel = motorForce/weight*gsToInchPerSecSquared;
accelLimit = gsToInchPerSecSquared*CoF;
kA = 12./maxAccel;
sysVoltage = inputVoltage - voltage/motorResistance*numMotors*robotResistance;

shiftVel = (kV(2)/kA(2)-kV(1)/kA(1))^-1*(inputVoltage*(1/kA(2)-1/kA(1))-(kC(2)/kA(2)-kC(1)/kA(1)));
%disp(shiftVel)

idx = 1;
% [t, pos, vel, accel, current, voltage, inputVoltage, sysVoltage]
SimulationResults = [0.0, 0.0, voltage-kC(1)-kV(1)*V0, ...
    (voltage - kC(1) - kV(1)*V0)/kA(1), voltage/motorResistance, voltage,...
    Vin, sysVoltage];

while (SimulationResults(idx,2) < targetDist && SimulationResults(idx, 1) < 60)
    Vin = min(inputVoltage, SimulationResults(idx, 7) + voltageRamp*dt);
    newTime = SimulationResults(idx, 1) + dt;
    newPos = SimulationResults(idx, 2) + dt*SimulationResults(idx, 3);
    newVel = SimulationResults(idx, 3) + dt*SimulationResults(idx, 4);
    
    if (newVel < shiftVel)
        shiftState = 1;
    else
        shiftState = 2;
    end
    
    sysVolt = SimulationResults(idx, 8);
    
    [newAccel, newVoltage, newCurrent, newSysVolt] = MechanismTimestep(...
        newVel, Vin, sysVolt, kC(shiftState), kV(shiftState), kA(shiftState),...
        currentLimit, accelLimit, inputVoltage, robotResistance, numMotors, ...
        motorResistance);
    
    idx = idx + 1;
    if (idx > size(SimulationResults, 1))
        SimResultsTemp = SimulationResults;
        SimulationResults = NaN(2*size(SimulationResults, 1), size(SimulationResults, 2));
        SimulationResults(1:idx-1, :) = SimResultsTemp(:,:);
    end
    
    SimulationResults(idx, :) = [newTime, newPos, newVel, newAccel,...
        newCurrent, newVoltage, Vin, newSysVolt];
end

SimulationResults = SimulationResults(1:idx, :);
SimulationResults = array2table(SimulationResults);
SimulationResults.Properties.VariableNames(1:8) = {'time', 'position', ...
    'velocity', 'acceleration', 'current', 'voltage', 'desiredVoltage', ...
    'systemVoltage'};
end