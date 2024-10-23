motionHistory = hSI.hMotionManager.motionHistory;
hMotionEstiomators = [motionHistory.hMotionEstimator];
channelMask = [hMotionEstiomators.channels] == 1;

motionHistory = motionHistory(channelMask);
c = vertcat(motionHistory.correlation);
zc = c(:,2);
zc = cellfun(@(zc)zc(:)',zc,'UniformOutput',false);
zc = vertcat(zc{:})';

figure;
plot(zc);
hold on;
plot(mean(zc,2),'LineWidth',3);