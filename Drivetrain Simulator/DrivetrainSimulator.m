function SimulationResults = ...
    DrivetrainSimulator(motor, numMotors, lowGear, highGear, wheelDiameter,...
    robotResistance, Ev, Et, weight, CoF, Rt, dt, V0, targetDist, inputVoltage, ...
    currentLimit, voltageRamp, oop)
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

if (~exist('oop', 'var'))
    oop = false;
end

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

shiftState = 1;

lastShift = -1;
preshiftV = 0;
shiftFreeze = 0.00;

if (oop)
%% Object Oriented
    sim = MechanismSimulator(kC(1), kV(1), kA(1), currentLimit, ...
        accelLimit, inputVoltage, robotResistance, numMotors, ...
        motorResistance);
    while (sim.getPos(end) < targetDist && sim.getTime(end) < 60)
        Vin = min(inputVoltage, sim.getVoltage(end) + voltageRamp*dt);
        if (sim.getVel(end) < shiftVel*.8 && shiftState == 2) % Shift Down
            shiftState = 1;
            sim.kC = kC(1);
            sim.kV = kV(1);
            sim.kA = kA(1);
            lastShift = sim.getTime(end);
            %fprintf('Shift down @ %.3f\n s', lastShift)
            preshiftV = Vin;
        elseif (sim.getVel(end) > shiftVel && shiftState == 1) % Shift Up
            shiftState = 2;
            sim.kC = kC(2);
            sim.kV = kV(2);
            sim.kA = kA(2);
            lastShift = sim.getTime(end);
            %fprintf('Shift up   @ %.3f\n s', lastShift)
            preshiftV = Vin;
        end
        
        if (sim.getTime(end) - lastShift < shiftFreeze)
            sim.step((sim.getTime(end)-lastShift)/shiftFreeze*preshiftV, dt);
        else
            sim.step(Vin, dt);
        end
    end
    SimulationResults = sim.getResults;
else
    %% Not Object Oriented
    idx = 1;
    simResults = [0,0,0,0,0,0,0,sysVoltage];
    while (simResults(idx,2) < targetDist && simResults(idx,1) < 60)
        Vin = min(inputVoltage, simResults(idx,7)+voltageRamp*dt);
        
        if (simResults(idx,3) < shiftVel)
            shiftState = 1;
        else
            shiftState = 2;
        end
        
        [newAccel, newVoltage, newCurrent, newSysVoltage] = MechanismTimestep(...
            simResults(idx,3), Vin, simResults(idx,8), kC(shiftState), kV(shiftState),...
            kA(shiftState), currentLimit, accelLimit, inputVoltage, robotResistance,...
            numMotors, motorResistance);
        
        newTime = simResults(idx,1) + dt;
        newPos = simResults(idx,2) + dt*simResults(idx,3);
        newVel = simResults(idx,3) + dt*newAccel;
        
        idx = idx+1;
        if (idx > size(simResults, 1))
            temp = simResults;
            simResults = NaN(size(simResults,1)*2, size(simResults,2));
            simResults(1:size(temp,1),:) = temp;
        end
        
        simResults(idx,:) = [newTime,newPos,newVel,newAccel,newCurrent,...
            newVoltage,Vin,newSysVoltage];
    end
    simResults = simResults(1:idx,:);
    SimulationResults = array2table(simResults);
    SimulationResults.Properties.VariableNames(1:8) = {'time', 'position', ...
        'velocity', 'acceleration', 'current', 'voltage', 'desiredVoltage', ...
        'systemVoltage'};
end
end