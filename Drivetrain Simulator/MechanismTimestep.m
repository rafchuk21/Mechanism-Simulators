function [newAccel, newVoltage, newCurrent, newSysVoltage] = ...
    MechanismTimestep(vel, Vin, sysVoltage, kC, kV, kA,...
    currentLimit, accelLimit, supplyVoltage, robotResistance,...
    numMotors, motorResistance)

Vv = kV * vel;
Vapp = min(Vin, sysVoltage);
newCurrent = (Vapp - Vv)/motorResistance;
newCurrent = min(newCurrent, currentLimit);
newCurrent = min(newCurrent, (kC + kA*accelLimit)/motorResistance);
newSysVoltage = supplyVoltage - newCurrent*numMotors*robotResistance;
newVoltage = motorResistance*newCurrent+Vv;
newAccel = (motorResistance*newCurrent-kC)/kA;

end