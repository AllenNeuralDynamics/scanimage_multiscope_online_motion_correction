classdef staticText < handle
    %UICONTROL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        hTxt;
        hAx;
        hPnl;
        hLis;
        
        % pass through to hCtl
        String;
        Visible;
        Style = 'text';
    end
    
    methods
        function obj = staticText(varargin)
            ip = most.util.InputParser;
            ip.addOptional('HorizontalAlignment', 'left');
            ip.addOptional('VerticalAlignment','middle');
            ip.addOptional('FontSize',8);
            ip.addOptional('FontWeight',[]);
            ip.addOptional('FontColor','k');
            ip.addOptional('String','');
            ip.addOptional('units','');
            ip.addOptional('BackgroundColor',[]);
            ip.addOptional('WidthLimits',[]);
            ip.addOptional('HeightLimits',[]);
            ip.addOptional('SizeLimits',[]);
            ip.parse(varargin{:});
            othrs = most.util.structPV2cellPV(ip.Unmatched);
            
            if ~isempty(ip.Results.units)
                obj.hPnl = uipanel('units',ip.Results.units,othrs{:},'BorderType','None');
            else
                obj.hPnl = uipanel(othrs{:},'BorderType','None');
            end
            
            if ~isempty(ip.Results.BackgroundColor)
                obj.hPnl.BackgroundColor = ip.Results.BackgroundColor;
            end
    
            if ~isempty(ip.Results.WidthLimits)
                lms = [ip.Results.WidthLimits ip.Results.WidthLimits(1)];
                set(obj.hPnl, 'WidthLimits', lms(1:2));
            end
            if ~isempty(ip.Results.HeightLimits)
                lms = [ip.Results.HeightLimits ip.Results.HeightLimits(1)];
                set(obj.hPnl, 'HeightLimits', lms(1:2));
            end
            if ~isempty(ip.Results.SizeLimits)
                set(obj.hPnl, 'WidthLimits', ip.Results.SizeLimits(1)*ones(1,2));
                set(obj.hPnl, 'HeightLimits', ip.Results.SizeLimits(2)*ones(1,2));
            end
            
            obj.hAx = most.idioms.axes('parent',obj.hPnl,'color','none','XTick',[],'XTickLabel',[],'YTick',[],'YTickLabel',[],'xcolor','none','ycolor','none','position',[0 0 1 1]);
            
            switch lower(ip.Results.HorizontalAlignment)
                case 'left'
                    pos = 0;
                case 'center'
                    pos = 0.5;
                case 'right'
                    pos = 1;
            end
            
            switch lower(ip.Results.VerticalAlignment)
                case 'top'
                    pos(2) = 1;
                case 'middle'
                    pos(2) = 0.55;
                case 'bottom'
                    pos(2) = 0;
            end
            
            obj.hTxt = text(pos(1),pos(2),0,ip.Results.String,'parent',obj.hAx,'HorizontalAlignment',ip.Results.HorizontalAlignment,'VerticalAlignment',ip.Results.VerticalAlignment,...
                'units','normalized','FontSize',ip.Results.FontSize,'Color',ip.Results.FontColor);
            
            if ~isempty(ip.Results.FontWeight)
                obj.hTxt.FontWeight = ip.Results.FontWeight;
            end
            
            obj.hLis = most.ErrorHandler.addCatchingListener(obj.hPnl.Parent,'ObjectBeingDestroyed',@(varargin)obj.delete);
        end
        
        function delete(obj)
            most.idioms.safeDeleteObj(obj.hLis);
            most.idioms.safeDeleteObj(obj.hTxt);
            most.idioms.safeDeleteObj(obj.hAx);
            most.idioms.safeDeleteObj(obj.hPnl);
        end
    end
    
    methods (Hidden)
        function set(obj,prop,val)
            switch lower(prop)
                case 'horizontalalignment'
                    pos = obj.hTxt.Position;
                    switch lower(val)
                        case 'left'
                            pos(0) = 0;
                        case 'center'
                            pos(0) = 0.5;
                        case 'right'
                            pos(0) = 1;
                    end
                    obj.hTxt.Position = pos;
                    obj.hTxt.(prop) = val;
                    
                case 'verticalalignment'
                    pos = obj.hTxt.Position;
                    switch lower(val)
                        case 'top'
                            pos(2) = 1;
                        case 'middle'
                            pos(2) = 0.5;
                        case 'bottom'
                            pos(2) = 0;
                    end
                    obj.hTxt.Position = pos;
                    obj.hTxt.(prop) = val;
                    
                case 'string'
                    obj.hTxt.(prop) = val;
                
                otherwise
                    set(obj.hPnl,prop,val);
            end
        end
        
        function v = get(obj,prop)
            if ismember(lower(prop), {'horizontalalignment' 'verticalalignment' 'string'})
                v = obj.hTxt.(prop);
            else
                v = obj.hAx.(prop);
            end
        end
    end
    
    methods
        function v = get.String(obj)
            v = obj.hTxt.String;
        end
        
        function set.String(obj,v)
            obj.hTxt.String = v;
        end
        
        function v = get.Visible(obj)
            v = obj.hPnl.Visible;
        end
        
        function set.Visible(obj,v)
            obj.hPnl.Visible = v;
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
