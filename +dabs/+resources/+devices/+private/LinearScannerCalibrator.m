classdef LinearScannerCalibrator < handle
    properties (Hidden)
        hLinearScanner;
        
        hFig;
        hAx;
        hTable;
        hEtLutIn;
        hEtLutOut;
        
        hCalPlotLinePts;
        hCalPlotLine;
        hCalPlotPt;
        
        hListeners = event.listener.empty(0,1);
    end
    
    %% Lifecycle
    methods
        function obj = LinearScannerCalibrator(hLinearScanner)
            obj.hLinearScanner = hLinearScanner;
            
            figName = sprintf('%s Position LUT',obj.hLinearScanner.name);
            obj.hFig = most.idioms.figure('CloseRequestFcn',@(varargin)obj.delete,'NumberTitle','off','MenuBar','none','Name',figName);
            obj.hFig.KeyPressFcn = @obj.KeyPressFcn;
            htop = most.idioms.uiflowcontainer('Parent',obj.hFig,'FlowDirection','LeftToRight');
            hmain = most.idioms.uiflowcontainer('Parent',htop,'FlowDirection','TopDown');
            
            hContextMenu = uicontextmenu();
            uimenu(hContextMenu,'Label','Delete','Callback',@(varargin)obj.deletePoint);
            
            up = uipanel('Parent',hmain,'bordertype','none');
            obj.hAx = most.idioms.axes('Parent',up,'FontSize',12,'FontWeight','Bold');
            box(obj.hAx,'on');
            obj.hCalPlotLine = line('Parent',obj.hAx,'XData',[],'YData',[],'Color',most.constants.Colors.black,'LineWidth',2,'Hittest','off','PickableParts','none');
            obj.hCalPlotLinePts = line('Parent',obj.hAx,'XData',[],'YData',[],'Color',most.constants.Colors.black,'Marker','o','LineStyle','none','LineWidth',1,'MarkerSize',7,'UIContextMenu',hContextMenu);
            obj.hCalPlotPt = line('Parent',obj.hAx,'XData',[],'YData',[],'Color',most.constants.Colors.red,'Marker','o','LineStyle','none','LineWidth',1,'MarkerSize',10,'Hittest','off','PickableParts','none');
            
            xlabel(obj.hAx,sprintf('Position LUT [%s]',obj.hLinearScanner.units),'FontWeight','Bold');
            ylabel(obj.hAx,sprintf('Position LUT [%s]',obj.hLinearScanner.units),'FontWeight','Bold');
            
            grid(obj.hAx,'on');
            title(obj.hAx,figName);
            
            bottomContainer = most.idioms.uiflowcontainer('Parent',hmain,'FlowDirection','LeftToRight');
            set(bottomContainer,'HeightLimits',[30 30]);
            
            uicontrol('parent',bottomContainer,'style','pushbutton','Callback',@(src,evt)obj.clearCal,'string','Reset');
            
            ad1 = uicontrol('parent',bottomContainer,'style','pushbutton','Callback',@(src,evt)obj.adjustCal(-0.03) ,'string',most.constants.Unicode.downwards_paired_arrow,'FontName','Arial Unicode MS');
            ad2 = uicontrol('parent',bottomContainer,'style','pushbutton','Callback',@(src,evt)obj.adjustCal(-0.005),'string',most.constants.Unicode.downwards_arrow,'FontName','Arial Unicode MS');
            ad3 = uicontrol('parent',bottomContainer,'style','pushbutton','Callback',@(src,evt)obj.adjustCal(0.005) ,'string',most.constants.Unicode.upwards_arrow,'FontName','Arial Unicode MS');
            ad4 = uicontrol('parent',bottomContainer,'style','pushbutton','Callback',@(src,evt)obj.adjustCal(0.03)  ,'string',most.constants.Unicode.upwards_paired_arrow,'FontName','Arial Unicode MS');
            set([ad1 ad2 ad3 ad4],'WidthLimits',[40 40]);
            
            hRightFlow = most.idioms.uiflowcontainer('Parent',htop,'FlowDirection','TopDown');
            hRightFlow.WidthLimits = [155 155];
            hTablePanel = uipanel('Parent',hRightFlow);
            hTableFlow = most.idioms.uiflowcontainer('Parent',hTablePanel,'FlowDirection','TopDown');
            
            obj.hTable = uitable('Parent',hTableFlow,'ColumnFormat',{'char','numeric','numeric'},'ColumnName',{'','LUT In','LUT Out'},'RowName',[],'ColumnEditable',[false,true,true],'ColumnWidth',{20,60,60},'CellEditCallback',@obj.tableEdited,'CellSelectionCallback',@obj.tableCellSelected);
            hAddEntryFlow = most.idioms.uiflowcontainer('Parent',hTableFlow,'FlowDirection','LeftToRight');
            hAddEntryFlow.HeightLimits = [25 25];
            most.gui.uicontrol('Parent',hAddEntryFlow,'style','pushbutton','Callback',@obj.addEntry,'string','+','WidthLimits',[20 20]);
            obj.hEtLutIn  = most.gui.uicontrol('Parent',hAddEntryFlow,'style','edit');
            obj.hEtLutOut = most.gui.uicontrol('Parent',hAddEntryFlow,'style','edit');
                        
            obj.hListeners(end+1) = addlistener(obj.hLinearScanner,'targetPosition','PostSet',@(varargin)obj.redraw);
            obj.hListeners(end+1) = addlistener(obj.hLinearScanner,'positionLUT',   'PostSet',@(varargin)obj.redraw);
            obj.hListeners(end+1) = addlistener(obj.hLinearScanner,'ObjectBeingDestroyed',@(varargin)obj.delete);
            obj.redraw();
        end
        
        function delete(obj)
            delete(obj.hListeners);
            most.idioms.safeDeleteObj(obj.hFig);
        end
    end
    
    methods        
        function redraw(obj)   
            obj.hAx.XLim = obj.hLinearScanner.travelRange;
            xx = linspace(obj.hLinearScanner.travelRange(1),obj.hLinearScanner.travelRange(2),100);
            yy = obj.hLinearScanner.lookUpPosition(xx);
            
            obj.hCalPlotLine.XData = xx;
            obj.hCalPlotLine.YData = yy;
            
            obj.hCalPlotLinePts.XData = obj.hLinearScanner.positionLUT(:,1);
            obj.hCalPlotLinePts.YData = obj.hLinearScanner.positionLUT(:,2);
            
            obj.hCalPlotPt.XData = obj.hLinearScanner.targetPosition;
            obj.hCalPlotPt.YData = obj.hLinearScanner.lookUpPosition(obj.hLinearScanner.targetPosition);
            
            obj.redrawTable();
        end
        
        function redrawTable(obj)
            oldData = obj.hTable.Data;
            
            lut = obj.hLinearScanner.positionLUT;
            
            data = num2cell(lut);
            X = most.constants.Unicode.ballot_x;
            data = [repmat({X},size(data,1),1),data];
            
            if ~isequal(oldData,data)
                obj.hTable.Data = data;
            end
        end
        
        function tableCellSelected(obj,src,evt)            
            if numel(evt.Indices)~=2 || evt.Indices(2)~=1
                return
            end
            
            idx = evt.Indices(1);
            lut = obj.hLinearScanner.positionLUT;
            lut(idx,:) = [];
            obj.hLinearScanner.positionLUT = lut;
            obj.applyNewLut();
        end
        
        function tableEdited(obj,src,evt)
            try
                data = obj.hTable.Data;
                lut = data(:,2:3);
                lut = cell2mat(lut);
                
                [~,idx] = unique(lut(:,1));
                lut = lut(idx,:);
                
                if ~isequal(obj.hLinearScanner.positionLUT,lut)
                    obj.hLinearScanner.positionLUT = lut;
                    obj.applyNewLut();
                end
            catch ME
                obj.redraw();
                most.ErrorHandler.logAndReportError(ME);
                errordlg(ME.message);
            end
        end
        
        function addEntry(obj,src,evt)
            try
                in  = obj.hEtLutIn.String;
                out = obj.hEtLutOut.String;
                
                in = str2double(in);
                out = str2double(out);
                
                lutEntry = [in out];
                
                lut = obj.hLinearScanner.positionLUT;
                lut(end+1,:) = lutEntry;
                
                [~,idx] = unique(lut(:,1));
                
                lut = lut(idx,:);
                obj.hLinearScanner.positionLUT = lut;
                
                obj.applyNewLut();
                
                obj.hEtLutIn.String  = '';
                obj.hEtLutOut.String = '';
            catch ME
                obj.redraw();
                most.ErrorHandler.logAndReportError(ME);
                errordlg(ME.message);
            end
        end
        
        function clearCal(obj)
            obj.hLinearScanner.positionLUT = [];
            obj.applyNewLut();
        end
        
        function deletePoint(obj)
            pt = obj.hAx.CurrentPoint(1,1:2);
            lut = obj.hLinearScanner.positionLUT;
            
            % find closest point in lut
            d = sqrt((lut(:,1)-pt(1)).^2 + (lut(:,2)-pt(2)).^2);
            [~,idx] = min(d);   
            
            lut(idx,:) = [];
            
            obj.hLinearScanner.positionLUT = lut;
            obj.applyNewLut();
        end
        
        function adjustCal(obj, adj)
            x = obj.hLinearScanner.targetPosition;            
            y = obj.hLinearScanner.lookUpPosition(x);
            
            lut = obj.hLinearScanner.positionLUT;            
            tolerance = diff(obj.hLinearScanner.travelRange) * 0.01;
            lut = removePointsInVicinity(lut,x,tolerance);            
            
            newX = x;
            newY = y+diff(obj.hLinearScanner.travelRange)*adj;

            lut(end+1,:) = [newX, newY];
            
            try
                obj.hLinearScanner.positionLUT = lut;
            catch ME
                most.ErrorHandler.logAndReportError(ME);
            end
            obj.applyNewLut();
            
            %%% Nested function
            function lut = removePointsInVicinity(lut,x,tolerance)
                mask = abs(lut(:,1)-x) <= tolerance;
                lut(mask,:) = [];
            end
        end
        
        function applyNewLut(obj)
            if isempty(obj.hLinearScanner.errorMsg)
                try
                    obj.hLinearScanner.pointPosition(obj.hLinearScanner.targetPosition);
                catch ME
                    most.ErrorHandler.logAndReportError(ME);
                end
            end
        end
        
        function KeyPressFcn(obj,src,evt)
            switch evt.Key
                case 'rightarrow'
                    obj.moveLinearScanner(0.005);
                case 'leftarrow'
                    obj.moveLinearScanner(-0.005);
                case 'uparrow'
                    obj.adjustCal(0.005);
                case 'downarrow'
                    obj.adjustCal(-0.005);
                case 'delete'
                    obj.deletePoint();
                otherwise
                    %No-op
            end
        end
        
        function moveLinearScanner(obj,adj)
            newPos = obj.hLinearScanner.targetPosition + diff(obj.hLinearScanner.travelRange) * adj;
            obj.hLinearScanner.pointPosition(newPos);
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
