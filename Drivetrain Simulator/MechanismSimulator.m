classdef MechanismSimulator < handle
    %MECHANISMSIMULATOR Simulates a mechanism
    
    properties
        timedDataMatrix
        kC
        kV
        kA
        currentLimit
        accelLimit
        supplyVoltage
    end
    
    properties(SetAccess = 'private')
        robotResistance
        numMotors
        motorResistance
    end
    
    methods
        function obj = MechanismSimulator(kC, kV, kA, currentLimit, ...
                accelLimit, supplyVoltage, robotResistance, numMotors, ...
                motorResistance)
            %MECHANISMSIMULATOR Construct an instance of this class
            obj.timedDataMatrix = [0,0,0,0,0,0,0,supplyVoltage];
            obj.kC = kC;
            obj.kV = kV;
            obj.kA = kA;
            obj.currentLimit = currentLimit;
            obj.accelLimit = accelLimit;
            obj.supplyVoltage = supplyVoltage;
            obj.robotResistance = robotResistance;
            obj.numMotors = numMotors;
            obj.motorResistance = motorResistance;
        end
        
        function step(obj, Vin, dt)
            %STEP Steps the simulation given an input voltage and a dt
            idx = size(obj.timedDataMatrix(~isnan(obj.timedDataMatrix(:,1)),1),1);
            newTime = obj.timedDataMatrix(idx,1) + dt;
            newPos = obj.timedDataMatrix(idx,2) + dt*obj.timedDataMatrix(idx,3);
            newVel = obj.timedDataMatrix(idx,3) + dt*obj.timedDataMatrix(idx,4);
            
            Vv = obj.kV * newVel;
            Vapp = min(Vin, obj.timedDataMatrix(idx,8));
            newCurrent = (Vapp - Vv)/obj.motorResistance;
            newCurrent = min(newCurrent, obj.currentLimit);
            newCurrent = min(newCurrent, (obj.kC + obj.kA*obj.accelLimit)/obj.motorResistance);
            newSysVoltage = min(obj.supplyVoltage, obj.supplyVoltage - newCurrent*obj.numMotors*obj.robotResistance);
            newVoltage = obj.motorResistance*newCurrent+Vv;
            if (abs(obj.motorResistance*newCurrent) < abs(obj.kC))
                newAccel = 0;
            else
                newAccel = (obj.motorResistance*newCurrent-obj.kC)/obj.kA;
            end
            
            idx = idx+1;
            
            if (idx > size(obj.timedDataMatrix, 1))
                newmat = NaN(2*size(obj.timedDataMatrix,1),size(obj.timedDataMatrix,2));
                newmat(1:size(obj.timedDataMatrix,1),:) = obj.timedDataMatrix;
                obj.timedDataMatrix = newmat;
            end
            obj.timedDataMatrix(idx,:) = [newTime, newPos, newVel, newAccel,...
                newCurrent, newVoltage, Vin, newSysVoltage];
            
%             obj.time = obj.addelement(obj.time, newTime, idx);
%             obj.pos = obj.addelement(obj.pos, newPos, idx);
%             obj.vel = obj.addelement(obj.vel, newVel, idx);
%             obj.accel = obj.addelement(obj.accel, newAccel, idx);
%             obj.current = obj.addelement(obj.current, newCurrent, idx);
%             obj.voltage = obj.addelement(obj.voltage, newVoltage, idx);
%             obj.desVoltage = obj.addelement(obj.desVoltage, Vin, idx);
%             obj.sysVoltage = obj.addelement(obj.sysVoltage, newSysVoltage, idx);
        end
        
        function v = getTime(obj, varargin)
            v = obj.timedDataMatrix(:,1);
            if (~isempty(varargin))
                v = v([varargin{:}]);
            else
                v = v(~isnan(v));
            end
        end
        
        function v = getPos(obj, varargin)
            v = obj.timedDataMatrix(:,2);
            if (~isempty(varargin))
                v = v([varargin{:}]);
            else
                v = v(~isnan(v));
            end
        end
        
        function v = getVel(obj, varargin)
            v = obj.timedDataMatrix(:,3);
            if (~isempty(varargin))
                v = v([varargin{:}]);
            else
                v = v(~isnan(v));
            end
        end
        
        function v = getAccel(obj, varargin)
            v = obj.timedDataMatrix(:,4);
            if (~isempty(varargin))
                v = v([varargin{:}]);
            else
                v = v(~isnan(v));
            end
        end
        
        function v = getCurrent(obj, varargin)
            v = obj.timedDataMatrix(:,5);
            if (~isempty(varargin))
                v = v([varargin{:}]);
            else
                v = v(~isnan(v));
            end
        end
        
        function v = getVoltage(obj, varargin)
            v = obj.timedDataMatrix(:,6);
            if (~isempty(varargin))
                v = v([varargin{:}]);
            else
                v = v(~isnan(v));
            end
        end
        
        function v = getDesVoltage(obj, varargin)
            v = obj.timedDataMatrix(:,7);
            if (~isempty(varargin))
                v = v([varargin{:}]);
            else
                v = v(~isnan(v));
            end
        end
        
        function v = getSysVoltage(obj, varargin)
            v = obj.timedDataMatrix(:,8);
            if (~isempty(varargin))
                v = v([varargin{:}]);
            else
                v = v(~isnan(v));
            end
        end
        
        function t = getResults(o)
            t = array2table(o.timedDataMatrix);
            t.Properties.VariableNames(1:8) = {'time', 'position', ...
                'velocity', 'acceleration', 'current', 'voltage', 'desiredVoltage', ...
                'systemVoltage'};
        end
        
    end
end

