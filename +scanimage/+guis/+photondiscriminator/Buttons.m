classdef Buttons < most.gui.GuiElement
    properties
        hPhotonDiscriminator;
        hBtGrabData;
        hBtSaveCfg;
        hBtLoadCfg;
        nDataPoints = 1e6;
        waitForFrameClk = false;
    end
    
    methods
        function obj = Buttons(hParent,hPhotonDiscriminator)
            obj = obj@most.gui.GuiElement(hParent);
            obj.hPhotonDiscriminator = hPhotonDiscriminator;
            obj.init();
        end
        
        function delete(obj)
            %No-OP
        end
        
        function init(obj)
            most.idioms.safeDeleteObj(obj.hBtGrabData);
            most.idioms.safeDeleteObj(obj.hBtSaveCfg);
            
            obj.hBtGrabData = uicontrol('Parent',obj.hUIPanel','Style','pushbutton','String','Grab Data','Callback',@(varargin)obj.grabData);
            obj.hBtSaveCfg = uicontrol('Parent',obj.hUIPanel','Style','pushbutton','String','Save Configuration','Callback',@(varargin)obj.saveCfg);
            obj.hBtLoadCfg = uicontrol('Parent',obj.hUIPanel','Style','pushbutton','String','Load Configuration','Callback',@(varargin)obj.loadCfg);
        end
        
        function scrollWheelFcn(obj,varargin)
           % No-Op 
        end
    end
    
    methods
        function configurationChanged(obj)
            
        end
        
        function panelResized(obj)
            panelPos = obj.getPositionInUnits('pixel');
            obj.hBtGrabData.Units = 'pixel';
            obj.hBtSaveCfg.Units = 'pixel';
            obj.hBtLoadCfg.Units = 'pixel';
            
            h = panelPos(4);
            padding = 10;
            buttonHeight = 30;
            obj.hBtGrabData.Position = [padding h-1.5*padding-buttonHeight panelPos(3)-2*padding, buttonHeight];
            obj.hBtSaveCfg.Position  = [padding h-2*padding-2*buttonHeight panelPos(3)-2*padding,buttonHeight];
            obj.hBtLoadCfg.Position  = [padding h-2.5*padding-3*buttonHeight panelPos(3)-2*padding,buttonHeight];
        end
        
        function grabData(obj)
            answer = inputdlg({'Number of data points','Trigger off frame clock'},'Grab Data',[1 30],{num2str(obj.nDataPoints),num2str(obj.waitForFrameClk)});
            if ~isempty(answer)
                nDataPoints_ = str2double(answer{1});
                validateattributes(nDataPoints_,{'numeric'},{'scalar','positive','nonnan','finite','integer','>=',1000});
                obj.nDataPoints = nDataPoints_;
                
                waitForFrameClk_ = str2double(answer{2});
                validateattributes(waitForFrameClk_,{'logical','numeric'},{'scalar','binary'});
                obj.waitForFrameClk = logical(waitForFrameClk_);
                
                silent = false;
                markPeriodClock = false;
                
                if obj.waitForFrameClk
                    trigger = 'Frame Clock';
                else
                    trigger = 'none';
                end
                
                if obj.hPhotonDiscriminator.simulated
                    data = floor(rand(obj.nDataPoints,1)*256 - 128);
                    obj.dataAcquired(data);
                else
                    obj.hPhotonDiscriminator.hSampler.acquireSampleDataSet(@obj.dataAcquired,obj.nDataPoints,trigger,markPeriodClock,silent);
                end
            end
        end
        
        function dataAcquired(obj,data)
            channel = obj.hPhotonDiscriminator.physicalChannelNumber;
            data = data(:,channel+1);
            obj.hParent.hDataPlot.updateData(data);            
        end
        
        function saveCfg(obj)
            [filename,pathname] = uiputfile('*.phtd','Save Photon Discriminator Configuration','DiscriminatorCfg.phtd');
            
            if ~isequal(filename,0)
                p = fullfile(pathname,filename);
                photonDiscriminatorConfig = obj.hPhotonDiscriminator.saveStruct();
                save(p,'photonDiscriminatorConfig');
            end
        end
        
        function loadCfg(obj)
            [filename,pathname] = uigetfile('*.phtd','Open Photon Discriminator Configuration');
            
            if ~isequal(filename,0)
                p = fullfile(pathname,filename);
                s = load(p,'-mat','photonDiscriminatorConfig');
                obj.hPhotonDiscriminator.loadStruct(s.photonDiscriminatorConfig);
            end
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
