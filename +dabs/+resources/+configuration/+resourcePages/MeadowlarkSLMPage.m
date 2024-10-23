classdef MeadowlarkSLMPage < dabs.resources.configuration.ResourcePage
    properties
        etLUT
        etRegionalLUT
    end
    
    methods
        function obj = MeadowlarkSLMPage(hResource,hParent)
            obj@dabs.resources.configuration.ResourcePage(hResource,hParent);
        end
        
        function makePanel(obj,hParent)            
            most.gui.uicontrol('Parent',hParent,'Style','text','RelPosition', [10 25 90 17],'Tag','txLUT','String','Look Up Table File','HorizontalAlignment','left');
            obj.etLUT = most.gui.uicontrol('Parent',hParent,'Style','edit','RelPosition', [10 45 290 20],'Tag','etLUT','HorizontalAlignment','left');
            most.gui.uicontrol('Parent',hParent,'RelPosition', [310 45 70 20],'String','Open','Tag','pbLUT','Callback',@(varargin)obj.openFile('*.*','lutFile'));
            
            if isprop(obj.hResource,'regionalLutFile')
                most.gui.uicontrol('Parent',hParent,'Style','text','RelPosition', [10 85 140 16],'Tag','txRegionalLUT','String','Regional Look Up Table File','HorizontalAlignment','left');
                obj.etRegionalLUT = most.gui.uicontrol('Parent',hParent,'Style','edit','RelPosition', [10 105 290 20],'Tag','etRegionalLUT','HorizontalAlignment','left');
                most.gui.uicontrol('Parent',hParent,'RelPosition', [310 105 70 20],'String','Open','Tag','pbRegionalLUT','Callback',@(varargin)obj.openFile('*.*','regionalLutFile'));
            end
        end
        
        function redraw(obj)      
            if isprop(obj.hResource,'regionalLutFile')
                obj.etRegionalLUT.String = obj.hResource.regionalLutFile;
            end
            
            obj.etLUT.String = obj.hResource.lutFile;
        end
        
        function apply(obj)
            if isprop(obj.hResource,'regionalLutFile')
                most.idioms.safeSetProp(obj.hResource,'regionalLutFile',obj.etRegionalLUT.String);
            end
            
            most.idioms.safeSetProp(obj.hResource,'lutFile',obj.etLUT.String);
            
            obj.hResource.saveMdf();
            obj.hResource.reinit();
        end
        
        function remove(obj)
            obj.hResource.deleteAndRemoveMdfHeading();
        end
        
        function openFile(obj,filter,propName)
            [file,path] = uigetfile(filter,'Select LUT File',obj.hResource.(propName));
            
            if isnumeric(file)
                return % user cancelled
            end
            
            file = fullfile(path,file);
            
            switch propName
                case 'lutFile'
                    obj.etLUT.String = file;
                case 'regionalLutFile'
                    obj.etRegionalLUT.String = file;
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
