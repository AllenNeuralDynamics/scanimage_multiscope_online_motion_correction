classdef AllenMotionCorrector < scanimage.interfaces.IMotionCorrector    
    properties (SetObservable)
        runningAverageLength_s = 3;     % running average length in seconds for motion correction value
        correctionInterval_s = 10;      % interval for motion correction events in secondes
        correctionThreshold = [1 1 10]; % units are in [degrees degrees micron]
        thresholdExceedTime_s = 0;
    end
    
    properties (SetObservable, Dependent)
        plotConfidence;
    end
    
    properties (SetAccess = private)
        motionHistory;  % stores the motion history
        lastUpdate;     % stores the time the last motion update happened
        drRefHistory = zeros(2,3,0);
        drRefHistoryTimestamps = zeros(1,0);
        drRefNeedsUpdate = true;
        lastDrRef = [];
        hConfidencePlot;
    end
        
    methods
        function obj = AllenMotionCorrector()
            obj.hConfidencePlot = most.util.TimePlot('Confidence Time Plot',false);
            obj.hConfidencePlot.xLabel = 'Acquisition Time [s]';
            obj.hConfidencePlot.yLabel = 'Confidence Value';
            obj.hConfidencePlot.changedCallback = @eigensetPlots;
            
            function eigensetPlots(varargin)
                % this is to update the user interface
                obj.plotConfidence = obj.plotConfidence;
            end
        end
        
        function delete(obj)
            delete(obj.hConfidencePlot);
        end
        
        function start(obj)
            obj.lastUpdate = tic();
            obj.drRefNeedsUpdate = true;
            obj.resetRefHistory();
            obj.hConfidencePlot.reset();
        end
        
        function abort(obj)
            % No-op
        end
        
        function updateMotionHistory(obj,motionHistory)
            % Note: in future releases the motionHistory might not be
            % sorted anymore. Use [motionHistory.historyIdx] to get the
            % history indices

            motionHistoryChannel = arrayfun(@(m)m.hMotionEstimator.channels,motionHistory);
            motionHistoryZ = [motionHistory.z];

            allZs = unique(motionHistoryZ);

            s = struct();
            for idx = 1:numel(allZs)
                z = allZs(idx);
                s(idx).z = z;
                s(idx).motionHistory_ch1 = motionHistory(motionHistoryChannel == 1 & motionHistoryZ == z);
                s(idx).motionHistory_ch2 = motionHistory(motionHistoryChannel == 2 & motionHistoryZ == z);
            end

            obj.motionHistory = s;

            obj.drRefNeedsUpdate = true;
            
            drRef = obj.getCorrection(); % update correction
            
            timeToUpdate = toc(obj.lastUpdate) > obj.correctionInterval_s;
            % if ~timeToUpdate
            %     return
            % end
            % 
            % historyMask = obj.drRefHistoryTimestamps >= obj.drRefHistoryTimestamps(end)-obj.thresholdExceedTime_s;
            % 
            % drRefs = obj.drRefHistory(historyMask,:);
            % threshExceed = false(1,3);
            % for axIdx = 1:3
            %     da = drRefs(:,axIdx);
            %     da = da(~isnan(da));
            %     threshExceed(axIdx) = all(abs(da) >= obj.correctionThreshold(axIdx));
            % end
            % aboveCorrectionThreshold = any(threshExceed);

            if timeToUpdate
                obj.lastUpdate = tic();
                obj.notify('correctNow');
            end
        end
        
        function drRef = getCorrection(obj)
            if ~obj.drRefNeedsUpdate
                drRef = obj.lastDrRef;
                return
            end

            s = obj.motionHistory;

            frameTimestamp = 0;

            nTimepoints = 0;

            correlationsZ_Ch1 = {};
            correlationsZ_Ch2 = {};

            for zidx = 1:numel(s)
                motionHistory_ch1 = s(zidx).motionHistory_ch1;
                motionHistory_ch2 = s(zidx).motionHistory_ch2;

                timestamps = [motionHistory_ch1.frameTimestamp];
                frameTimestamp = max(frameTimestamp,max(timestamps));

                historymask = timestamps>=timestamps-obj.runningAverageLength_s;
                nTimepoints = sum(historymask);

                motionHistory_ch1 = motionHistory_ch1(historymask);
                motionHistory_ch2 = motionHistory_ch2(historymask);

                drRefs_ch1 = vertcat(motionHistory_ch1.drRef);  % [nTimepoints,3]
                drRefs_ch2 = vertcat(motionHistory_ch2.drRef);  % [nTimepoints,3]

                drRef_ch1 = mean(drRefs_ch1,1,'omitnan');
                drRef_ch2 = mean(drRefs_ch2,1,'omitnan');

                s(zidx).drRef_ch1 = drRef_ch1;
                s(zidx).drRef_ch2 = drRef_ch2;

                correlations1 = vertcat(motionHistory_ch1.correlation);
                correlations2 = vertcat(motionHistory_ch2.correlation);
                correlationsZ_Ch1 = vertcat(correlationsZ_Ch1, correlations1(:,3));
                correlationsZ_Ch2 = vertcat(correlationsZ_Ch2, correlations2(:,3));
            end

            correlationsZ_Ch1 = vertcat(correlationsZ_Ch1{:});
            correlationsZ_Ch2 = vertcat(correlationsZ_Ch2{:});

            if nTimepoints == 0
                drRef = zeros(2,3);
            else
                % calculate the per actuator correction
                drRefs_ch1 = vertcat(s.drRef_ch1);
                drRefs_ch2 = vertcat(s.drRef_ch2);

                drRef_ch1 = mean(drRefs_ch1,1,'omitnan');
                drRef_ch2 = mean(drRefs_ch2,1,'omitnan');

                % ensure x,y are the same
                xy = mean([drRef_ch1(1:2);drRef_ch2(1:2)],1);
                drRef_ch1(1:2) = xy;
                drRef_ch2(1:2) = xy;

                correlationsZ_Ch1 = mean(correlationsZ_Ch1,1);
                correlationsZ_Ch2 = mean(correlationsZ_Ch2,1);

                [~,idx1] = max(correlationsZ_Ch1);
                [~,idx2] = max(correlationsZ_Ch2);

                % assume numel(correlationsZ_Ch1) is odd
                centerIdx1 = (numel(correlationsZ_Ch1)-1)/2+1;
                centerIdx2 = (numel(correlationsZ_Ch2)-1)/2+1;
                
                delta1 = idx1 - centerIdx1;
                delta2 = idx2 - centerIdx2;

                dZ1 = delta1 * -0.75;
                dZ2 = delta2 * -0.75;

                drRef = [drRef_ch1;drRef_ch2];

                drRef(1,3) = dZ1;
                drRef(2,3) = dZ2;

                obj.drRefHistory(:,:,end+1) = drRef;
                obj.drRefHistoryTimestamps(end+1) = frameTimestamp;
            end
            
            obj.lastDrRef = drRef;
            obj.drRefNeedsUpdate = false;
            
            % % update confidence values
            % if obj.plotConfidence
            %     historymask = timestamps == timestamp;
            %     confidence = {obj.motionHistory.confidence};
            %     confidence = vertcat(confidence{:});
            %     confidence = mean(confidence,1,'omitnan');
            %     obj.hConfidencePlot.addTimePoint(confidence,timestamp);
            % end
        end
        
        function correctedMotion(obj,dr,motionCorrectionVector)
            %No-op
        end
    end
    
    %% Internal methods
    methods (Access = private)        
        function resetRefHistory(obj)
            obj.drRefHistory = zeros(2,3,0);
            obj.drRefHistoryTimestamps = zeros(1,0);
        end
    end
    
    %% Property Getter/Setter
    methods
        % JK
        function lastUpdate = get_lastUpdate(obj)
            lastUpdate = obj.lastUpdate;
        end
        
        % JK
        function lastTimeStamp = get_lastTimeStamp(obj)
            lastTimeStamp = obj.lastTimeStamp;
        end

        function set.runningAverageLength_s(obj,val)
            validateattributes(val,{'numeric'},{'scalar','positive','nonnan','real','finite'});
            obj.runningAverageLength_s = val;
        end
        
        function set.correctionInterval_s(obj,val)
            validateattributes(val,{'numeric'},{'scalar','positive','nonnan','real','finite'});
            obj.correctionInterval_s = val;
        end
        
        function set.correctionThreshold(obj,val)
            validateattributes(val,{'numeric'},{'vector','row','numel',3,'nonnegative','nonnan','real'});
            obj.correctionThreshold = val;
        end
        
        function set.plotConfidence(obj,val)
            validateattributes(val,{'numeric','logical'},{'scalar','binary'});
            obj.hConfidencePlot.visible = logical(val);
        end
        
        function val = get.plotConfidence(obj)
            val = obj.hConfidencePlot.visible;
        end
    end
end

function vec = append(vec,v,veclength)
if isempty(vec)
    vec = v;
elseif size(vec,1) < veclength
    vec(end+1,:) = v;
else
    if size(vec,1) > veclength
        vec = vec(end-veclength+1:end,:);
    end
    vec = circshift(vec,-1,1);
    vec(end,:) = v;
end
end




% ----------------------------------------------------------------------------
% Copyright (C) 2021 Vidrio Technologies, LLC
% 
% ScanImage (R) 2021 is software to be used under the purchased terms
% Code may be modified, but not redistributed without the permission
% of Vidrio Technologies, LLC
% 
% VIDRIO TECHNOLOGIES, LLC MAKES NO WARRANTIES, EXPRESS OR IMPLIED, WITH
% RESPECT TO THIS PRODUCT, AND EXPRESSLY DISCLAIMS ANY WARRANTY OF
% MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.
% IN NO CASE SHALL VIDRIO TECHNOLOGIES, LLC BE LIABLE TO ANYONE FOR ANY
% CONSEQUENTIAL OR INCIDENTAL DAMAGES, EXPRESS OR IMPLIED, OR UPON ANY OTHER
% BASIS OF LIABILITY WHATSOEVER, EVEN IF THE LOSS OR DAMAGE IS CAUSED BY
% VIDRIO TECHNOLOGIES, LLC'S OWN NEGLIGENCE OR FAULT.
% CONSEQUENTLY, VIDRIO TECHNOLOGIES, LLC SHALL HAVE NO LIABILITY FOR ANY
% PERSONAL INJURY, PROPERTY DAMAGE OR OTHER LOSS BASED ON THE USE OF THE
% PRODUCT IN COMBINATION WITH OR INTEGRATED INTO ANY OTHER INSTRUMENT OR
% DEVICE.  HOWEVER, IF VIDRIO TECHNOLOGIES, LLC IS HELD LIABLE, WHETHER
% DIRECTLY OR INDIRECTLY, FOR ANY LOSS OR DAMAGE ARISING, REGARDLESS OF CAUSE
% OR ORIGIN, VIDRIO TECHNOLOGIES, LLC's MAXIMUM LIABILITY SHALL NOT IN ANY
% CASE EXCEED THE PURCHASE PRICE OF THE PRODUCT WHICH SHALL BE THE COMPLETE
% AND EXCLUSIVE REMEDY AGAINST VIDRIO TECHNOLOGIES, LLC.
% ----------------------------------------------------------------------------
