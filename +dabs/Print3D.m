classdef Print3D < dabs.resources.Device & dabs.resources.widget.HasWidget & most.HasMachineDataFile & dabs.resources.configuration.HasConfigPage
    properties (SetAccess = protected)
        WidgetClass = 'dabs.Print3DWidget';
    end
    
    %% ABSTRACT PROPERTY REALIZATIONS (most.HasMachineDataFile) 
    properties (Constant, Hidden)
        %Value-Required properties
        mdfClassName = mfilename('class');
        mdfHeading = 'Print3D';
        
        %Value-Optional properties
        mdfDependsOnClasses; %#ok<MCCPI>
        mdfDirectProp;       %#ok<MCCPI>
        mdfPropPrefix;       %#ok<MCCPI>
        
        mdfDefault = defaultMdfSection();
    end
    
    properties (SetObservable)
        imageStack = {};
        numRepeats = 1;
        zStep_um = 1;
        startScript;
        endScript;
        stackMode = scanimage.types.StackMode.slow;
    end
    
    properties (SetObservable, SetAccess = private)
        slicesDone = 0;
        hListeners = event.listener.empty();
        hWaitbar = [];
        
        active = false;
        startPosition = 0;
    end
    
    properties (Dependent, Hidden)
        hSI
        hFastZ
    end
    
    properties (SetAccess = protected)
        ConfigPageClass = 'dabs.resources.configuration.resourcePages.BlankPage';
    end
    
    methods (Static)
        function names = getDescriptiveNames()
            names = {'Print 3D'};
        end
    end
    
    methods
        function obj = Print3D(name)
            obj@dabs.resources.Device(name);
            obj = obj@most.HasMachineDataFile(true);
            
            obj.deinit();
            obj.loadMdf();
            obj.reinit();
        end
        
        function delete(obj)
            obj.deinit();
        end
    end
    
    methods
        function reinit(obj)
            try
                obj.deinit();
                obj.errorMsg = '';
            catch ME
                obj.deinit();
                obj.errorMsg = sprintf('%s: initialization error: %s',obj.name,ME.message);
                most.ErrorHandler.logError(ME,obj.errorMsg);
            end
        end
        
        function deinit(obj)
            obj.errorMsg = 'Uninitialized';
        end
    end
    
    methods
        function loadMdf(obj)
            success = true;
            success = success & obj.safeSetPropFromMdf('startScript', 'startScript');
            success = success & obj.safeSetPropFromMdf('endScript', 'endScript');
            
            if ~success
                obj.errorMsg = 'Error loading config';
            end
        end
        
        function saveMdf(obj)
            obj.safeWriteVarToHeading('startScript', obj.startScript);
            obj.safeWriteVarToHeading('endScript', obj.endScript);
        end
    end
    
    methods
        function start(obj)            
            assert(most.idioms.isValidObj(obj.hSI),'ScanImage is not initialized');
            assert(strcmpi(obj.hSI.acqState,'idle'),'ScanImage is currently imaging');
            assert(~isempty(obj.imageStack),'No images loaded');
            
            obj.abort();
            
            obj.hSI.hStackManager.enable = false;
            obj.hSI.hStackManager.framesPerSlice = obj.numRepeats;
            obj.hSI.extTrigEnable = false;
            obj.hSI.hRoiManager.mroiEnable = false;
            
            if isa(obj.hSI.hScan2D,'scanimage.components.scan2d.RggScan')
                obj.hSI.hScan2D.sampleRateCtlMax = 2e6;
                obj.hSI.hScan2D.sampleRateCtl = 2e6;
            end
            
            if isprop(obj.hSI.hScan2D,'keepResonantScannerOn')
                obj.hSI.hScan2D.keepResonantScannerOn = true;
            end
            
            if obj.stackMode == scanimage.types.StackMode.slow
                obj.hSI.hMotors.queryPosition();
                obj.startPosition = obj.hSI.hMotors.samplePosition;
            else
                obj.startPosition = obj.hFastZ.targetPosition;
            end
            
            obj.hListeners(end+1) = most.ErrorHandler.addCatchingListener(obj.hSI.hUserFunctions,'acqModeDone',@(varargin)obj.nextSlice);
            
            obj.hWaitbar = waitbar(0,sprintf('Progress: 0/%d',numel(obj.imageStack)),'Name','Ablating...','CreateCancelBtn',@(varargin)obj.abort);
            
            obj.active = true;
            obj.slicesDone = -1;
            
            if ~isempty(obj.startScript)
                try
                    evalin('base',obj.startScript);
                catch ME
                    most.ErrorHandler.logAndReportError(ME,['Error occurred running start script: ' ME.message]);
                end
            end
            
            obj.nextSlice();
        end
        
        function nextSliceDelayed(obj)
            % this is necessary because of the weird calling order that
            % happens with the user events that indicate a complete
            % acquisition.
            
            hTimer = timer('Name','Delayed Nxt Slice');
            hTimer.ExecutionMode = 'SingleShot';
            hTimer.StartDelay = 0.01;
            hTimer.TimerFcn = @(varargin)timerFcn(hTimer);
            
            function timerFcn(hTimer)
                most.idioms.safeDeleteObj(hTimer);
                obj.nextSlice();
            end
        end
        
        function nextSlice(obj)
            if ~obj.active
                return
            end      
            
            obj.slicesDone = obj.slicesDone + 1;
            
            waitbar(obj.slicesDone/numel(obj.imageStack),obj.hWaitbar,sprintf('Progress: %d/%d',obj.slicesDone,numel(obj.imageStack)));
            
            if obj.slicesDone == numel(obj.imageStack)
                obj.abort();
                return
            end
            
            % set up power box
            powerbox = struct( ...
                 'rect'     ,[0 0 1 1]  ...
                ,'powers'   ,NaN        ...
                ,'name'     ,'Ablation' ...
                ,'oddLines' ,true       ...
                ,'evenLines',true       ...
                ,'mask'     ,obj.imageStack{obj.slicesDone + 1} ...
                );
            
            obj.hSI.hBeams.powerBoxes = powerbox;
            obj.hSI.hBeams.enablePowerBox = true;
            
            dz = obj.zStep_um * obj.slicesDone;
            
            if obj.stackMode == scanimage.types.StackMode.slow
                nextZ = obj.startPosition(3) + dz;
                nextMotorPosition = [obj.startPosition(1:2) nextZ];
                obj.hSI.hMotors.moveSample(nextMotorPosition);
            else
                assert(most.idioms.isValidObj(obj.hFastZ),'No FastZ configured for scan system ''%s''',obj.hSI.hScan2D.name);
                nextZ = obj.startPosition + dz;
                obj.hFastZ.moveBlocking(nextZ);
            end
            
            try
                obj.hSI.startGrab();                
            catch ME
                obj.abort();
                most.ErrorHandler.logAndReportError(ME);
            end
        end
        
        function abort(obj)
            wasActive = obj.active;
            obj.active = false;
            
            delete(obj.hListeners);
            obj.hListeners = event.listener.empty();
            
            most.idioms.safeDeleteObj(obj.hWaitbar);
            
            if most.idioms.isValidObj(obj.hSI)
                obj.hSI.abort();
                
                if isprop(obj.hSI.hScan2D,'keepResonantScannerOn')
                    obj.hSI.hScan2D.keepResonantScannerOn = false;
                end
                
                obj.hSI.hBeams.enablePowerBox = false;
            end
            
            if wasActive && ~isempty(obj.endScript)
                try
                    evalin('base',obj.endScript);
                catch ME
                    most.ErrorHandler.logAndReportError(ME,['Error occurred running end script: ' ME.message]);
                end
            end
        end
        
        function loadImageStack(obj,fileNames)
            if nargin < 2 || isempty(fileNames)
                [files,paths] = uigetfile('*.*','Select image files','MultiSelect','on');
                
                if isnumeric(files)
                    return
                end
                
                if ~iscell(files)
                    files = {files};
                    paths = {paths};
                end
                
                fileNames = fullfile(paths,files);
            end
            
            imageStack_ = cell(numel(fileNames),1);
            
            hWb = waitbar(0,'Loading images');
            
            try
                for idx = 1:numel(fileNames)
                    [data,map,transparency] = imread(fileNames{idx});
                    
                    if isinteger(data)
                        dataClass = class(data);
                        dataRange = single( [intmin(dataClass) intmax(dataClass)] );
                        
                        data = single(data);
                        data = (data - dataRange(1)) ./ diff(dataRange);
                    end
                    
                    data = mean(data,3);
                    
                    imageStack_{idx} = data;
                    
                    waitbar(idx/numel(fileNames),hWb);
                end
            catch ME
                most.idioms.safeDeleteObj(hWb);
                ME.rethrow();
            end
            
            most.idioms.safeDeleteObj(hWb);
            
            obj.imageStack = imageStack_;
        end
        
        function showImages(obj)
            if isempty(obj.imageStack)
                h = helpdlg('No images are loaded');
                most.gui.centerOnScreen(h);
                return
            end
            
            hListener = most.ErrorHandler.addCatchingListener(obj,'ObjectBeingDestroyed',@(varargin)close);
            hListener = most.ErrorHandler.addCatchingListener(obj,'imageStack','PostSet',@(varargin)close);
            hFig = figure('NumberTitle','off','Menubar','none','Name','Ablation Images','CloseRequestFcn',@(varargin)close,'WindowScrollWheelFcn',@scroll);
            
            hAx = axes('Parent',hFig);
            
            imageIdx = 1;
            showImage();
            
            function scroll(src,evt)
                imageIdx = imageIdx + sign(evt.VerticalScrollCount);
                showImage();
            end            
            
            function showImage()
                numImages = numel(obj.imageStack);
                imageIdx = max(min(numImages,imageIdx),1);
                imagesc(hAx,obj.imageStack{imageIdx});
                axis(hAx,'image');
                colormap(hFig,gray);
                hAx.CLim = [0,1];
                hAx.XTick = [];
                hAx.YTick = [];
                title(hAx,sprintf('%d/%d',imageIdx,numImages));
                xlabel('Use scroll wheel to flip through images');
            end
            
            function close()
                most.idioms.safeDeleteObj(hListener);
                most.idioms.safeDeleteObj(hFig);
            end
        end
    end
    
    methods
        function set.imageStack(obj,val)
            if isempty(val)
                val = {};
            else
                validateattributes(val,{'cell'},{'vector'});
                for idx = 1:numel(val)
                    validateattributes(val{idx},{'single','double'},{'2d','>=',0,'<=',1});
                end
            end
            
            obj.imageStack = val;
        end
        
        function set.numRepeats(obj,val)
            validateattributes(val,{'numeric'},{'positive','integer','scalar','nonnan','finite','real'});
            obj.numRepeats = val;
        end
        
        function set.zStep_um(obj,val)
            validateattributes(val,{'numeric'},{'scalar','nonnan','finite','real'});
            assert(val~=0,'zStep_um most not be zero');
            obj.zStep_um = val;
        end
        
        function val = get.hSI(obj)
            val = obj.hResourceStore.filterByClass('scanimage.SI');
            if isempty(val) || ~val{1}.mdlInitialized
                val = [];
            else
                val = val{1};
            end
        end
        
        function val = get.hFastZ(obj)
            hFastZs = obj.hSI.hScan2D.hFastZs;
            
            if isempty(hFastZs)
                val = [];
            else
                val = hFastZs{1};
            end
        end
        
        function set.startScript(obj,v)
            validateattributes(v,{'char'},{});
            obj.startScript = v;
        end
        
        function set.endScript(obj,v)
            validateattributes(v,{'char'},{});
            obj.endScript = v;
        end
        
        function set.stackMode(obj,val)
            val = most.idioms.string2Enum(val,'scanimage.types.StackMode'); 
            validateattributes(val,{'scanimage.types.StackMode'},{'scalar'});
            
            if obj.stackMode ~= val
                assert(~obj.active,'Cannot change stack mode while printing is active');
                obj.stackMode = val;
            end
        end
    end
end

function s = defaultMdfSection()
s = [...
    most.HasMachineDataFile.makeEntry('startScript', '','Script that runs at the beginning of the Print')...
    most.HasMachineDataFile.makeEntry('endScript',   '','Script that runs at the end of the Print')...
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
