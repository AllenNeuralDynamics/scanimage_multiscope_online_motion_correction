hSI.hMotionManager.clearAndDeleteEstimators();
% hMotionEstimatorCh1 = makeMotionEstimator(1);
% hMotionEstimatorCh2 = makeMotionEstimator(2);
% hSI.hMotionManager.addEstimator(hMotionEstimatorCh1);
% hSI.hMotionManager.addEstimator(hMotionEstimatorCh2);

% zs_all = hSI.hRoiManager.currentRoiGroup.activeRois(1).zs;
% zs_ch1 = zs_all(5:8);
% zs_ch2 = zs_all(1:4);

zs_ch1 = hSI.hStackManager.zsAllActuators(:,1); % 5:8
zs_ch2 = hSI.hStackManager.zsAllActuators(:,2); % 4:1

% 84
% 114
% 158
% 192

for i = 1: length(zs_ch1)
    z = zs_ch1(i);
    zScan = zs_ch1(i);
    hMotionEstimatorCh1 = makeMotionEstimator(1,z,zScan);
    hSI.hMotionManager.addEstimator(hMotionEstimatorCh1);
end

% 64
% 20
% -18
% -60

% just to update the UI
hSI.hMotionManager.estimatorClassName = 'scanimage.components.motionEstimators.MariusMotionEstimator';

for i = 1: length(zs_ch2)
    z = zs_ch2(i);
    zScan = zs_ch1(i);
    hMotionEstimatorCh2 = makeMotionEstimator(2,z,zScan);
    % hMotionEstimatorCh2 = makeMotionEstimator(1,z);  % 05/21/2024 - JK: Current limit in the motion estimator - just channel 1?
    hSI.hMotionManager.addEstimator(hMotionEstimatorCh2);
end


hSI.hMotionManager.enable = true;