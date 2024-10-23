classdef SLM < dabs.resources.devices.SLM & dabs.resources.configuration.HasConfigPage & most.HasMachineDataFile
    properties (SetAccess = protected)
        ConfigPageClass = 'dabs.resources.configuration.resourcePages.GenericSLMPage';
    end
    
    methods (Static)
        function names = getDescriptiveNames()
            names = {'SLM\Simulated SLM'};
        end
    end
    
    %%% ABSTRACT PROPERTY REALIZATIONS (most.HasMachineDataFile)
    properties (Constant, Hidden)
        %Value-Required properties
        mdfClassName = mfilename('class');
        mdfHeading = 'Simulated SLM';

        %Value-Optional properties
        mdfDependsOnClasses; %#ok<MCCPI>
        mdfDirectProp;       %#ok<MCCPI>
        mdfPropPrefix;       %#ok<MCCPI>
        
        mdfDefault = defaultMdfSection();
    end
    
    %%% Abstract property realizations (scanimage.mroi.scanners.SLM)
    properties (Constant)
        queueAvailable = false;
    end
    
    %% LifeCycle    
    methods
        function obj = SLM(name)
            obj@dabs.resources.devices.SLM(name);
            obj@most.HasMachineDataFile(true);
            
            obj.pixelBitDepth = 8;
            obj.computeTransposedPhaseMask = true;
            
            obj.deinit();
            obj.loadMdf();
            obj.reinit();
        end
        
        function delete(obj)
            obj.deinit();
        end
    end
    
    methods        
        function deinit(obj)
            obj.errorMsg = 'uninitialized';
        end
        
        function reinit(obj)
            obj.deinit();
            
            try
                obj.errorMsg = '';
            catch ME
                obj.deinit();
                obj.errorMsg = sprintf('%s: initialization error: %s',obj.name,ME.message);
                most.ErrorHandler.logError(ME,obj.errorMsg);
            end            
        end
    end
    
    methods        
        function loadMdf(obj)
            success = true;
            success = success & obj.safeSetPropFromMdf('pixelResolutionXY', 'pixelResolutionXY');
            success = success & obj.safeSetPropFromMdf('pixelPitchXY', 'pixelPitchXY', @(v)v/1e6); % conversion from microns to meter
            success = success & obj.safeSetPropFromMdf('maxRefreshRate', 'maxRefreshRate');
            
            if ~success
                obj.errorMsg = 'Error loading config';
            end
        end
        
        function saveMdf(obj)            
            obj.safeWriteVarToHeading('pixelResolutionXY', obj.pixelResolutionXY);
            obj.safeWriteVarToHeading('pixelPitchXY',      obj.pixelPitchXY * 1e6); % conversion from meter to microns
            obj.safeWriteVarToHeading('maxRefreshRate',    obj.maxRefreshRate);
        end
    end
    
    %% User Methods
    methods
        function writeBitmap(obj,phaseMaskRaw,waitForTrigger)
            assert(isempty(obj.errorMsg),obj.errorMsg)
            
            if obj.computeTransposedPhaseMask
                assert(isequal(size(phaseMaskRaw),obj.pixelResolutionXY));
            else
                assert(isequal(size(phaseMaskRaw),flip(obj.pixelResolutionXY)));
            end
            
            obj.lastKnownBitmap = cast(phaseMaskRaw,obj.pixelDataType);
        end
    end
    
    methods (Access = protected)
        function resizeSlmQueue(obj,length)
            error('Unsupported');
        end
        
        function writeSlmQueue(obj,frames)
            error('Unsupported');
        end
        
        function startSlmQueue(obj)
            error('Unsupported');
        end
        
        function abortSlmQueue(obj)
            error('Unsupported');
        end
    end
end

function s = defaultMdfSection()
    s = [...
        most.HasMachineDataFile.makeEntry('pixelResolutionXY',[1920,1080],'[x,y] pixel resolution of SLM')...
        most.HasMachineDataFile.makeEntry('pixelPitchXY',[6.4 6.4],'[1x2 numeric] distance from pixel center to pixel center in microns')...
        most.HasMachineDataFile.makeEntry()...
        most.HasMachineDataFile.makeEntry('maxRefreshRate',60)...
        ];
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
