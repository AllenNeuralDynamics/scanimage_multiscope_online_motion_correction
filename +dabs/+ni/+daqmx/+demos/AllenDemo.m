function [hTask,hAOTask] = AllenDemo()

%%%%EDIT IF NEEDED%%%%
AIDevice = 'PXI1Slot3';
AIChans = 0:1; %Must be 2 channels
AODevice = 'PXI1Slot3';
AOChan = 0; %Must be 1 channel

sampleRate = 2e6; %Hz
updatePeriod = 1; %s
acqPeriod = 5;
%%%%%%%%%%%%%%%%%%%%%%

import dabs.ni.daqmx.*

updatePeriodSamples = round(updatePeriod * sampleRate);

taskNames = {'AllenAI' 'AllenAO'};
taskMap = Task.getTaskMap();
for i=1:length(taskNames)    
   if taskMap.isKey(taskNames{i})
       delete(taskMap(taskNames{i}));
   end
end    
    
hTask = Task('AllenAI');
hAOTask = Task('AllenAO');

hTask.createAIVoltageChan(AIDevice,AIChans,[],-10,10);
hAOTask.createAOVoltageChan(AODevice,AOChan);
hTask.registerDoneEvent(@AllenDoneCallback);

numUpdates = round(round(sampleRate*acqPeriod)/updatePeriodSamples);
acqNumSamples = numUpdates * updatePeriodSamples;
hTask.cfgSampClkTiming(sampleRate,'DAQmx_Val_FiniteSamps', acqNumSamples);

hTask.registerEveryNSamplesEvent(@AllenCallback,updatePeriodSamples,false,'native');

% hTask.everyNSamples = updatePeriodSamples;
% hTask.everyNSamplesEventCallbacks = @AllenCallback;
% 
% hTask.everyNSamplesReadDataEnable = false;
% hTask.everyNSamplesReadDataTypeOption = 'native';


tic;
hAOTask.start();
hTask.start();

    function AllenCallback(src,evnt)        
        
        persistent hFig
        
        if isempty(hFig)
            hFig = figure();
        end
        
        if src.everyNSamplesReadDataEnable
            displayDataInfo(evnt.data);
        
            if isempty(evnt.errorMessage)
                figure(hFig);
                plot(evnt.data);
            end
        else
            %tic;
            inData = readAnalogData(src,src.everyNSamples,'native');   
            %toc;

            displayDataInfo(inData);
            
            figure(hFig);
            plot(inData);      
        end
        
        function displayDataInfo(dataSrc)
            fprintf(1,'Read Data Info -- size: %s\tmean: %.5g\tmax: %g\tmin: %g\tclass: %s\n',mat2str(size(dataSrc)),...
                mean(dataSrc(:)), max(dataSrc(:)), min(dataSrc(:)),class(dataSrc));
        end
    end

    function AllenDoneCallback(src,evnt)
       disp('Acquisition Done!');      
      
       delete([hTask hAOTask]);        
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
