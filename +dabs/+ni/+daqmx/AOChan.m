classdef AOChan < dabs.ni.daqmx.private.AnalogChan
    %AOCHAN A DAQmx Analog Output Channel
    
    properties (Constant)
        type = 'AnalogOutput';
    end
    
    properties (Constant, Hidden)
        typeCode = 'AO';
    end
    
    %% CONSTRUCTOR/DESTRUCTOR
    methods        
        function obj = AOChan(varargin) 
            %%%TMW: Constructor required, as this is a concrete subclass of abstract lineage
            obj = obj@dabs.ni.daqmx.private.AnalogChan(varargin{:});                
        end                       
    end
    
    %% METHODS
    methods (Hidden=true)
        
        function postCreationListener(obj)
            %Determine raw data types for channel(s) which have been added to the Task via the Channel specification
            
            errorCond = false;
            for i=1:length(obj)
                resolution = obj(i).getQuiet('resolution');
                if resolution <= 8
                    rawSampClass = 'int8';
                elseif resolution <=16 
                    rawSampClass = 'int16';
                elseif resolution <=32
                    rawSampClass = 'int32';
                else
                    errorCond = true;
                    break;
                end
                    
                if isempty(obj(i).task.rawDataArrayAI)
                    obj(1).task.rawDataArrayAI = feval(rawSampClass,0); %Creates a scalar array of rawSampClass
                elseif ~strcmpi(class(obj(i).task.rawDataArrayAI), rawSampClass);
                    errMessage = ['All ' obj(i).type ' channels in a given Task must have the same raw data type. Task deleted.'];
                    errorCond = true;
                    break;
                end
            end
            
            if errorCond
                delete(obj(1).task); %All created objects presumed (known) to belong to same class
                error(errMessage);
            end

        end
     
       
        %
        %         function postCreationListener(obj)
        %             %Determine raw data types for device(s) which have been added to the Task via the Channel specification
        %
        %             task = obj(1).task;
        %             typeCode = obj(1).typeCode; %#ok<PROP>
        %
        %             devices = task.devices;
        %             rawDataClasses = {devices.(['rawDataClass' typeCode])}; %#ok<*PROP> %Force cell array output
        %             if ~all(strcmpi(rawDataClasses{1},rawDataClasses)) || ...
        %                     (~isempty(task.(['rawDataArray' typeCode])) && ~strcmpi(class(task.(['rawDataArray' typeCode])), rawDataClasses{1}))
        %                 most.idioms.dispError('ERROR: At this time, a Task can only support multiple devices with same raw AI and AO data formats\n');
        %                 delete(task); %Give up on the Task! TODO: Consider just removing the incorrectly added Channels
        %                 return;
        %             else
        %                 task.(['rawDataArray' typeCode]) = feval(rawDataClasses{1},0); %Creates scalar array of desired class
        %             end
        %         end
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
