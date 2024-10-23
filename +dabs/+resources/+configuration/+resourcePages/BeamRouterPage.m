classdef BeamRouterPage < dabs.resources.configuration.ResourcePage
    properties
        tablehBeams
        etFunctionHandle
        etFunctionHandleCalibration
    end
    
    methods
        function obj = BeamRouterPage(hResource,hParent)
            obj@dabs.resources.configuration.ResourcePage(hResource,hParent);
        end
        
        function makePanel(obj,hParent)
            obj.tablehBeams    = most.gui.uicontrol('Parent',hParent,'Style','uitable','ColumnFormat',{'char','logical'},'ColumnEditable',[false,true],'ColumnName',{'Beam','Use'},'ColumnWidth',{80 30},'RowName',[],'RelPosition', [10 223 115 110],'Tag','tablehBeams');

            most.gui.uicontrol('Parent',hParent,'Style','text','RelPosition', [10 23 100 17],'Tag','txFunctionHandle','String','Function handle','HorizontalAlignment','left');
            obj.etFunctionHandle = most.gui.uicontrol('Parent',hParent,'Style','edit','String','','RelPosition', [10 42 310 20],'Tag','etFunctionHandle','HorizontalAlignment','left');
            most.gui.uicontrol('Parent',hParent,'String','Edit','RelPosition', [330 42 40 20],'Tag','pbEditFunction','Callback',@(varargin)obj.editFunction);
            
            most.gui.uicontrol('Parent',hParent,'Style','text','RelPosition', [10 73 130 20],'Tag','txFunctionHandleCalibration','String','Calibration function handle','HorizontalAlignment','left');
            obj.etFunctionHandleCalibration = most.gui.uicontrol('Parent',hParent,'Style','edit','String','','RelPosition', [10 88 310 20],'Tag','etFunctionHandleCalibration','HorizontalAlignment','left');
            most.gui.uicontrol('Parent',hParent,'String','Edit','RelPosition', [330 88 40 20],'Tag','pbEditCalibrationFunction','Callback',@(varargin)obj.editCalibrationFunction);
            
            most.gui.uicontrol('Parent',hParent,'String','Use example functions','Callback',@(varargin)obj.useExampleFunctions,'RelPosition', [230 132 140 30],'Tag','pbExamples');
        end
        
        function redraw(obj)            
            allBeams = obj.hResourceStore.filterByClass('dabs.resources.devices.BeamModulatorFast');
            allBeamNames = cellfun(@(hR)hR.name,allBeams,'UniformOutput',false);
            beamNames = cellfun(@(hR)hR.name,obj.hResource.hBeams,'UniformOutput',false);
            selected = ismember(allBeamNames,beamNames);
            obj.tablehBeams.Data = most.idioms.horzcellcat(allBeamNames,num2cell(selected));

            obj.etFunctionHandle.String = function2String(obj.hResource.functionHandle);            
            obj.etFunctionHandleCalibration.String = function2String(obj.hResource.functionHandleCalibration);

            function str = function2String(func)
                if isempty(func)
                    str = '';
                else
                    str = func2str(func);
                    if ~strcmp(str(1),'@')
                        str = ['@' str];
                    end
                end
            end
        end
        
        function apply(obj)
            beamNames = obj.tablehBeams.Data(:,1)';
            selected   = [obj.tablehBeams.Data{:,2}];
            beamNames = beamNames(selected);
            most.idioms.safeSetProp(obj.hResource,'hBeams',beamNames);
            
            most.idioms.safeSetProp(obj.hResource,'functionHandle',obj.etFunctionHandle.String);
            most.idioms.safeSetProp(obj.hResource,'functionHandleCalibration',obj.etFunctionHandleCalibration.String);
            
            obj.hResource.saveMdf();
            obj.hResource.reinit();
        end
        
        function useExampleFunctions(obj)
            try
                obj.hResource.useExampleFunctions();
                obj.redraw();
                obj.editFunction();
                obj.editCalibrationFunction();
            catch ME
                most.ErrorHandler.logAndReportError(ME);
                obj.redraw();
            end
        end
        
        function editFunction(obj)
            funcStr = obj.etFunctionHandle.String;
            obj.editFunctionString(funcStr);
            drawnow();
            obj.raise();
        end
        
        function editCalibrationFunction(obj)
            funcStr = obj.etFunctionHandleCalibration.String;
            obj.editFunctionString(funcStr);
            drawnow();
            obj.raise();    
        end
        
        function editFunctionString(obj,funcStr)
            funcStr = regexprep(funcStr,'^@(\([^\)]*\))?','');
            funcStr = regexprep(funcStr,'(\([^\)]*\))$','');
            
            funcLocation = which(funcStr);
            
            if ~isempty(funcLocation) && isempty(strfind(funcLocation,'built-in'))
                edit(funcLocation);
            else
                warndlg(sprintf('Function ''%s'' not found on disk',funcStr));
            end
        end
        
        function remove(obj)
            obj.hResource.deleteAndRemoveMdfHeading();
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
