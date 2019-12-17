numTrials = 30;
runTimes = NaN(numTrials,1);
fprintf('[')
for i = 1:numTrials
    if (mod(i,numTrials/10) == 0)
        fprintf('|');
    end
    startTime = rem(now,1)*24*3600*1000; % start time in ms
    r = DrivetrainSimulator('MiniCIM', 6, 10.8, 4.6, 4, .02, .9, .9, 154, 1.1, 7, .001, 0, 240, 12, 60, 120, false);
    runTimes(i) = rem(now,1)*24*3600*1000-startTime;
end
fprintf(']\n%f\n', median(runTimes))