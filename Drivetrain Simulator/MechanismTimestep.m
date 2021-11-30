% Simulates a characterized mechanism's behavior. Can work with voltage
% sag, current limit, acceleration limit.
%
% vel: Current system velocity
% Vin: Desired applied motor voltage
% sysVoltage: System voltage (post voltage sag)
% kC: Intercept voltage
% kV: Velocity voltage coefficient (V_vel = back emf = kV*vel)
% kA: Acceleration voltage coefficient (V_accel = kA * accel)
% currentLimit: Motor current limit
% accelLimit: System acceleration limit
% supplyVoltage: Idle (pre-voltage-sag) battery voltage
% robotResistance: Robot resistance in Ohms (includes battery resistance)
% numMotors: Number of motors powering the system
% motorResistance: Motor resistance in Ohms
function [newAccel, newVoltage, newCurrent, newSysVoltage] = ...
    MechanismTimestep(vel, Vin, sysVoltage, kC, kV, kA,...
    currentLimit, accelLimit, supplyVoltage, robotResistance,...
    numMotors, motorResistance)

% Compute back emf voltage
Vv = kV * vel;
% Limit applied voltage to be no more than the system voltage
Vapp = absmin(Vin, sysVoltage*sign(Vin));
% Compute the current present if this voltage were applied
newCurrent = (Vapp - Vv)/motorResistance;
% If current exceeds current limit, cap to current limit
newCurrent = absmin(newCurrent, currentLimit*sign(newCurrent));
% If current corresponds to accel > max accel, cap to max accel current
newCurrent = absmin(newCurrent, (kC + kA*accelLimit)/motorResistance*sign(newCurrent));
% Find the voltage that would give the desired current
newVoltage = motorResistance*newCurrent+Vv; % If no limits, newVoltage = Vapp
% Compute the system voltage after voltage sag
if (newVoltage * newCurrent < 0)
    voltageSag = -abs(newCurrent*numMotors*robotResistance);
else
    voltageSag = abs(newCurrent*numMotors*robotResistance);
end
newSysVoltage = supplyVoltage - voltageSag;
% Compute the new acceleration of the system
newAccel = (newVoltage-Vv-kC)/kA;

end

% Returns whichever input has a lower absolute value
function m = absmin(n1, n2)
if abs(n1) <= abs(n2)
    m = n1;
else
    m = n2;
end
end