classdef SIMotorsPage < dabs.resources.configuration.ResourcePage
    properties
        pmhMotorX
        pmhMotorY
        pmhMotorZ
        
        etScaleX
        etScaleY
        etScaleZ
    end
    
    methods
        function obj = SIMotorsPage(hResource,hParent)
            obj@dabs.resources.configuration.ResourcePage(hResource,hParent);
        end
        
        function makePanel(obj,hParent)
            most.gui.uicontrol('Parent',hParent,'Style','text','RelPosition', [90 22 100 20],'Tag','txMotorLabel','String','Motor assignment','HorizontalAlignment','left');
            most.gui.uicontrol('Parent',hParent,'Style','text','RelPosition', [320 22 70 20],'Tag','txScaling','String','Scaling','HorizontalAlignment','left');
            
            most.gui.uicontrol('Parent',hParent,'Style','text','RelPosition', [-2 46 80 20],'Tag','txMotorX','String','Sample X Axis','HorizontalAlignment','right');
            most.gui.uicontrol('Parent',hParent,'Style','text','RelPosition', [-2 76 80 20],'Tag','txMotorY','String','Sample Y Axis','HorizontalAlignment','right');
            most.gui.uicontrol('Parent',hParent,'Style','text','RelPosition', [-2 106 80 20],'Tag','txMotorZ','String','Sample Z Axis','HorizontalAlignment','right');
            
            obj.pmhMotorX = most.gui.uicontrol('Parent',hParent,'Style','popupmenu','String',{''},'RelPosition', [80 42 230 20],'Tag','pmhMotorX');
            obj.pmhMotorY = most.gui.uicontrol('Parent',hParent,'Style','popupmenu','String',{''},'RelPosition', [80 72 230 20],'Tag','pmhMotorY');
            obj.pmhMotorZ = most.gui.uicontrol('Parent',hParent,'Style','popupmenu','String',{''},'RelPosition', [80 102 230 20],'Tag','pmhMotorZ');
            
            obj.etScaleX = most.gui.uicontrol('Parent',hParent,'Style','edit','String','','RelPosition', [320 42 40 19],'Tag','etScaleX');
            obj.etScaleY = most.gui.uicontrol('Parent',hParent,'Style','edit','String','','RelPosition', [320 72 40 19],'Tag','etScaleY');
            obj.etScaleZ = most.gui.uicontrol('Parent',hParent,'Style','edit','String','','RelPosition', [320 102 40 19],'Tag','etScaleZ');
            
            panel = most.gui.uicontrol('Parent',hParent,'Style','uipanel','RelPosition', [10 291 360 181],'Tag','panel');
            dabs.resources.configuration.resourcePages.private.stageCoordinates(panel.hCtl);
        end
        
        function redraw(obj)
            obj.hResource.validateConfiguration();
            
            hMotors = obj.hResourceStore.filterByClass('dabs.resources.devices.MotorController');
            
            axisNames = {};
            for motorIdx = 1:numel(hMotors)
                hMotor = hMotors{motorIdx};
                for axIdx = 1:hMotor.numAxes
                    axisNames{end+1} = sprintf('%s - motor %d',hMotor.name,axIdx);
                end
            end
            
            obj.pmhMotorX.String = [{''}, axisNames];
            obj.pmhMotorX.pmValue = sprintf('%s - motor %d',obj.hResource.hMotorXYZ{1}.name,obj.hResource.motorAxisXYZ(1));
            
            obj.pmhMotorY.String = [{''}, axisNames];
            obj.pmhMotorY.pmValue = sprintf('%s - motor %d',obj.hResource.hMotorXYZ{2}.name,obj.hResource.motorAxisXYZ(2));
            
            obj.pmhMotorZ.String = [{''}, axisNames];
            obj.pmhMotorZ.pmValue = sprintf('%s - motor %d',obj.hResource.hMotorXYZ{3}.name,obj.hResource.motorAxisXYZ(3));
            
            obj.etScaleX.String = num2str(obj.hResource.scaleXYZ(1));
            obj.etScaleY.String = num2str(obj.hResource.scaleXYZ(2));
            obj.etScaleZ.String = num2str(obj.hResource.scaleXYZ(3));
        end
        
        function apply(obj)
            [motorNameX,motorAxisX] = parseMotorPullDown(obj.pmhMotorX,1);
            [motorNameY,motorAxisY] = parseMotorPullDown(obj.pmhMotorY,2);
            [motorNameZ,motorAxisZ] = parseMotorPullDown(obj.pmhMotorZ,3);
            
            most.idioms.safeSetProp(obj.hResource,'hMotorXYZ',{motorNameX,motorNameY,motorNameZ});
            most.idioms.safeSetProp(obj.hResource,'motorAxisXYZ',[motorAxisX,motorAxisY,motorAxisZ]);
            
            most.idioms.safeSetProp(obj.hResource,'scaleXYZ' ...
                ,[str2double(obj.etScaleX.String) ...
                 ,str2double(obj.etScaleY.String) ...
                 ,str2double(obj.etScaleZ.String) ...
                 ]);
             
            obj.hResource.saveMdf();
            obj.hResource.reinit();
            obj.hResource.validateConfiguration();
            
            %%% Nested function
            function [motorName,motorAxis] = parseMotorPullDown(pmhMotor,defaultAxis)
                if isempty(pmhMotor.pmValue)
                    motorName = '';
                    motorAxis  = defaultAxis;
                else
                    match = regexpi(pmhMotor.pmValue,'(.*) - motor ([0-9]+)','tokens','once');
                    motorName = match{1};
                    motorAxis  = str2double(match{2});
                end
            end
        end
        
        function remove(obj)
            % No-op
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
