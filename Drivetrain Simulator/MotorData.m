function motorData = MotorData()
% Returns motor data in a table.

colNames = {'Falcon', 'NEO', 'MiniCIM', 'CIM', 'Redline', 'BAG'};
rowNames = {'Free Speed', 'Stall Torque', 'Stall Current', 'Free Current'};

falcon = [6380, 4.69, 257, 1.5]';
neo = [5880, 3.36, 166, 1.3]';
minicim = [5840, 1.41, 89, 3]';
cim = [5330, 2.41, 131, 2.7]';
redline = [18730, .71, 134, .7]';
bag = [13180, .43, 53, 1.8]';

motorData = table(falcon, neo, minicim, cim, redline, bag, 'RowNames', rowNames, 'VariableNames', colNames);
end

