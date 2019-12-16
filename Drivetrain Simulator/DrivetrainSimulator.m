function SimulationResults = ...
    DrivetrainSimulator(motor, numMotors, lowGear, highGear, wheelDiameter,...
    robotResistance, Ev, Et, weight, Rt, dt, V0, targetDist, inputVoltage, ...
    currentLimit, voltageRamp)

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
kA = 12./maxAccel;
sysVoltage = inputVoltage - voltage/motorResistance*numMotors*robotResistance;

shiftVel = (kV(2)/kA(2)-kV(1)/kA(1))^-1*(inputVoltage*(1/kA(2)-1/kA(1))-(kC(2)/kA(2)-kC(1)/kA(1)));
%disp(shiftVel)

idx = 1;
% [t, pos, vel, Vv, Va, accel, current, voltage, inputVoltage]
SimulationResults = [0.0, 0.0, V0, kV(1)*V0, voltage-kC(1)-kV(1)*V0, ...
    (voltage - kC(1) - kV(1)*V0)/kA(1), voltage/motorResistance, voltage,...
    Vin, sysVoltage];

while (SimulationResults(idx,2) < targetDist && SimulationResults(idx, 1) < 60)
    % newVoltage = inputVoltage - SimulationResults(idx,7)*numMotors*robotResistance;
    Vin = min(inputVoltage, SimulationResults(idx, 9) + voltageRamp*dt);
    newCurrent = min(currentLimit,(Vin - SimulationResults(idx, 8)) / (numMotors * robotResistance));
    newSysVolt = inputVoltage - newCurrent * numMotors * robotResistance;
    newTime = SimulationResults(idx, 1) + dt;
    newPos = SimulationResults(idx, 2) + dt*SimulationResults(idx, 3);
    newVel = SimulationResults(idx, 3) + dt*SimulationResults(idx, 6);
    
    if (newVel < shiftVel)
        shiftState = 1;
    else
        shiftState = 2;
    end
    
    newVoltage = min(Vin, (inputVoltage+numMotors*robotResistance*...
        kV(shiftState)/motorResistance*newVel)/(1+numMotors*robotResistance...
        /motorResistance)); % V(t) = min(voltageRamp, systemVoltage)
    newVv = kV(shiftState)*newVel;
    newVa = newVoltage - kC(shiftState) - newVv;
    newAccel = newVa/kA(shiftState);
    %newCurrent = abs(kC(shiftState)+newVa)/motorResistance;
    
    idx = idx + 1;
    if (idx > size(SimulationResults, 1))
        SimResultsTemp = SimulationResults;
        SimulationResults = NaN(2*size(SimulationResults, 1), size(SimulationResults, 2));
        SimulationResults(1:idx-1, :) = SimResultsTemp(:,:);
    end
    
    SimulationResults(idx, :) = [newTime, newPos, newVel, newVv, newVa,...
        newAccel, newCurrent, newVoltage, Vin, newSysVolt];
end

SimulationResults = SimulationResults(1:idx, :);

end