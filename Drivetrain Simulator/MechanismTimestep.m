function [newAccel, newVoltage, newCurrent, newSysVoltage] = ...
    MechanismTimestep(vel, Vin, sysVoltage, kC, kV, kA,...
    currentLimit, accelLimit, supplyVoltage, robotResistance,...
    numMotors, motorResistance)

Vv = kV * vel;
Vapp = min(Vin, sysVoltage);
if Vapp < kC
    kC = -Vapp;
end
newCurrent = (Vapp - Vv)/motorResistance;
newCurrent = absmin(newCurrent, currentLimit);
newCurrent = absmin(newCurrent, (abs(kC) + kA*accelLimit)/motorResistance);
newSysVoltage = supplyVoltage - abs(newCurrent*numMotors*robotResistance);
newVoltage = motorResistance*newCurrent+Vv;
newAccel = (Vapp-Vv-kC)/kA;

end

function m = absmin(n1, n2)
if abs(n1) <= abs(n2)
    m = n1;
else
    m = n2;
end
end