function [newAccel, newVoltage, newCurrent, newSysVoltage] = ...
    MechanismTimestep(vel, Vin, sysVoltage, kC, kV, kA,...
    currentLimit, accelLimit, supplyVoltage, robotResistance,...
    numMotors, motorResistance)

Vv = kV * vel;
Vapp = absmin(Vin, sysVoltage*sign(Vin));
newCurrent = (Vapp - Vv)/motorResistance;
newCurrent = absmin(newCurrent, currentLimit*sign(newCurrent));
newCurrent = absmin(newCurrent, (kC + kA*accelLimit)/motorResistance*sign(newCurrent));
newVoltage = motorResistance*newCurrent+Vv; % If no limits, newVoltage = Vapp
newSysVoltage = supplyVoltage - abs(newCurrent*numMotors*robotResistance);
newAccel = (newVoltage-Vv-kC)/kA;

end

function m = absmin(n1, n2)
if abs(n1) <= abs(n2)
    m = n1;
else
    m = n2;
end
end