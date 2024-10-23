classdef SLMFrameQueue < handle
    properties (Dependent, SetAccess = private)
        iterationIdx
        queueIdx
    end
    
    properties (SetAccess = private)
        numBytesPerFrame = 0;
        queueLength = 0;
        running = false;
    end
    
    properties (Access = private)
        hFrameQueue;
    end
    
    methods
        function obj = SLMFrameQueue(slmDeviceHandle,numBytesPerFrame)
            assert(isa(slmDeviceHandle,'uint64'),'Expect slmDeviceHandle to be of type uint64');
            obj.hFrameQueue = SlmFrameQueue('make',slmDeviceHandle);
            obj.numBytesPerFrame = numBytesPerFrame;
        end
        
        function delete(obj)
            if obj.running
                obj.abort();
            end
            SlmFrameQueue('delete',obj.hFrameQueue);
        end
    end
    
    methods        
        function start(obj)
            assert(obj.queueLength>0, 'Frame queue is empty');
            assert(~obj.running,'Frame queue is already running');
            obj.running = true;
            SlmFrameQueue('start',obj.hFrameQueue);
        end
        
        function abort(obj)
            SlmFrameQueue('abort',obj.hFrameQueue);
            obj.running = false;
        end
        
        function write(obj,data)
            if iscell(data)
                nFrames = numel(data);
                assert( all( cellfun( @(d) isa(d,'uint8') && iscolumn(d) ,data) ),'Data in cell array must be column vector of type uint8');
                framesizes = cellfun(@(d)numel(d),data);
                data = vertcat(data{:});
            else
                nFrames = size(data,3);
                data = data(:);
                data = typecast(data,'uint8');
                assert(numel(data) == nFrames * obj.numBytesPerFrame,'Data size mismatch');
                framesizes = repmat(obj.numBytesPerFrame,nFrames,1);
            end
            obj.queueLength = nFrames; 
            framesizes = uint64(framesizes);
            SlmFrameQueue('write',obj.hFrameQueue,data,framesizes);
        end
    end
    
    methods (Access = private)
        function [running,iterationIdx,queueIdx] = getStatus(obj)
            [running,iterationIdx,queueIdx] = SlmFrameQueue('getStatus',obj.hFrameQueue);
            iterationIdx = iterationIdx+1;
            queueIdx = queueIdx+1;
        end
    end
    
    %% Property Getter/Setter
    methods
        function val = get.iterationIdx(obj)
            [~,iterationIdx_,~] = obj.getStatus();
            val = iterationIdx_;
        end
        
        function val = get.queueIdx(obj)
            [~,~,queueIdx_] = obj.getStatus();
            val = queueIdx_;
        end
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
