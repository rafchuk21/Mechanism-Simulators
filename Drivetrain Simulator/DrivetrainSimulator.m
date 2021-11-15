function SimulationResults = ...
    DrivetrainSimulator(motor, numMotors, lowGear, highGear, wheelDiameter,...
    robotResistance, Ev, Et, weight, CoF, Rt, dt, V0, targetDist, inputVoltage, ...
    currentLimit, voltageRamp)
% Simulates the behavior of a drivetrain given a "full ahead" voltage.
% Accounts for voltage sag and implements current limiting and voltage
% ramping. Does not come to rest at target distance and does not account
% for wheel slip. Can theoretically give the simulator any sequence of
% voltage inputs, but no guarantee that it actually works for slowing down.
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

% Conversion factors
nmToInLb = 8.85074579;
gsToInchPerSecSquared = 386.09;

radius = wheelDiameter/2;

% Get relevant motor data
motorData = MotorData();
freeSpeed = motorData.(motor)(1);
stallTorque = motorData.(motor)(2);
stallCurrent = motorData.(motor)(3);
freeCurrent = motorData.(motor)(4);

% Initial value for input voltage (could be useful idk)
Vin = 0.0;

% Gearbox stall torque
gbStallTorque = stallTorque*nmToInLb*numMotors*Et*[lowGear,highGear];
% Motor resistance
motorResistance = 12/stallCurrent;
% not sure tbh
voltage = Vin*motorResistance/(motorResistance+numMotors*robotResistance);
% Theoretical 12V free speed at each gearing
twelveVoltSpeed = freeSpeed/60*pi*wheelDiameter*Ev*[1/lowGear, 1/highGear];
% Value for intercept voltage
kC = 12*Rt./gbStallTorque;
% Value for kV (V/(in/s))
kV = (12-kC)./twelveVoltSpeed;
% Maximum acceleration that the gearbox can provide
maxAccel = gbStallTorque/radius/weight*gsToInchPerSecSquared;
% Maximum acceleration that traction allows for (a_max = F_N*CoF/m = g*CoF)
accelLimit = gsToInchPerSecSquared*CoF;
% Value for kA (V/(in/s^2))
kA = 12./maxAccel;
%fprintf('kC: %.3f, kV: %.3f, kA: %.3f\n', kC(1), kV(1), kA(1));
%fprintf('kC: %.3f, kV: %.3f, kA: %.3f\n', kC(2), kV(2), kA(2));
% System voltage (post voltage-sag) for initial input voltage
sysVoltage = inputVoltage - voltage/motorResistance*numMotors*robotResistance;

% Optimal shift velocity or something
shiftVel = (kV(2)/kA(2)-kV(1)/kA(1))^-1*(inputVoltage*(1/kA(2)-1/kA(1))-(kC(2)/kA(2)-kC(1)/kA(1)));
%disp(shiftVel)

% start in low gear
shiftState = 1;

%% Not Object Oriented
idx = 1;
simResults = [0,0,V0,0,0,0,0,sysVoltage];
stopping = false;

while (simResults(idx, 1) < 5 && simResults(idx,2) < targetDist)
    % Can write different driving input - here just simulates driver giving
    % full 12V signal constantly.
    Vb = fullthrottle(); %Vb - base input voltage
    
    % Gets the voltage from the last timestep
    Vlast = simResults(idx, 7);
    % Enforce the voltage ramp
    if (Vlast - voltageRamp * dt > Vb)
        Vdesired = Vlast - voltageRamp*dt;
    elseif (Vlast + voltageRamp * dt < Vb)
        Vdesired = Vlast + voltageRamp*dt;
    else
        Vdesired = Vb;
    end
    
    % Limit the motor voltage to no more than the input voltage
    Vin = min(inputVoltage, Vdesired);
    
    % If in high gear and slower than the shifting speed or if trying to
    % stop, downshift.
    if (shiftState == 2 && (simResults(idx,3) < shiftVel || stopping))
        shiftState = 1;
        %disp("Shifting Down");
    % If in low gear and higher than the shift speed, upshift.
    elseif (shiftState == 1 && simResults(idx,3) > shiftVel)
        shiftState = 2;
        %disp("Shifting Up");
    end
    
    % Use the average of the last 20 system voltage (post-voltage-sag)
    sysVoltageAvg = mean(simResults(max(idx-20,1):max(idx-1,1),8));
    
    % Compute the timestep of the drive using the current drive state and
    % inputs.
    [newAccel, newVoltage, newCurrent, newSysVoltage] = MechanismTimestep(...
        simResults(idx,3), Vin, sysVoltageAvg, ...
        kC(shiftState)*sign(simResults(idx,3)), kV(shiftState),...
        kA(shiftState), currentLimit, accelLimit, inputVoltage, robotResistance,...
        numMotors, motorResistance);
    
    % Update simulation values
    newTime = simResults(idx,1) + dt;
    newPos = simResults(idx,2) + dt*simResults(idx,3);
    newVel = simResults(idx,3) + dt*(newAccel+simResults(max(idx-1,1),4))/2;
    
    % Increment the index. If simulation result table is full, double its
    % size.
    idx = idx+1;
    if (idx > size(simResults, 1))
        temp = simResults;
        simResults = NaN(size(simResults,1)*2, size(simResults,2));
        simResults(1:size(temp,1),:) = temp;
    end
    
    % Store simulation timestep results.
    simResults(idx,:) = [newTime,newPos,newVel,newAccel,newCurrent,...
        newVoltage,Vin,newSysVoltage];
end
% Trim simulation results, convert to a table, assign table names.
% SimulationResults gets returned by the function.
simResults = simResults(1:idx,:);
SimulationResults = array2table(simResults);
SimulationResults.Properties.VariableNames(1:8) = {'time', 'position', ...
    'velocity', 'acceleration', 'current', 'voltage', 'desiredVoltage', ...
    'systemVoltage'};
end

function V = fullthrottle()
V = 12;
end