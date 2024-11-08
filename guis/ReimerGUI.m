function varargout = ReimerGUI(varargin)
% REIMERGUI MATLAB code for ReimerGUI.fig
%      REIMERGUI, by itself, creates a new REIMERGUI or raises the existing
%      singleton*.
%
%      H = REIMERGUI returns the handle to a new REIMERGUI or the handle to
%      the existing singleton*.
%
%      REIMERGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in REIMERGUI.M with the given input arguments.
%
%      REIMERGUI('Property','Value',...) creates a new REIMERGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ReimerGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ReimerGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ReimerGUI

% Last Modified by GUIDE v2.5 08-Mar-2017 16:23:56

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ReimerGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @ReimerGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before ReimerGUI is made visible.
function ReimerGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ReimerGUI (see VARARGIN)

% Choose default command line output for ReimerGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes ReimerGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ReimerGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in cbDivertSamples.
function cbDivertSamples_Callback(hObject, eventdata, handles)
% hObject    handle to cbDivertSamples (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbDivertSamples
hFpga = gethFpga();
hFpga.LinScanDivertSamples = hObject.Value;

% --- Executes on slider movement.
function slTriggerDelay_Callback(hObject, eventdata, handles)
% hObject    handle to slTriggerDelay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
hFpga = gethFpga();
hFpga.LinScanLaserTriggerDelay = hObject.Value;

% --- Executes during object creation, after setting all properties.
function slTriggerDelay_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slTriggerDelay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



% --- Executes on slider movement.
function slNumberOfSamples_Callback(hObject, eventdata, handles)
% hObject    handle to slNumberOfSamples (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
hFpga = gethFpga();
hFpga.LinScanNumberOfSamplesToDivert = hObject.Value;

% --- Executes during object creation, after setting all properties.
function slNumberOfSamples_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slNumberOfSamples (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function etLaserTriggerFilter_Callback(hObject, eventdata, handles)
% hObject    handle to etLaserTriggerFilter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of etLaserTriggerFilter as text
%        str2double(get(hObject,'String')) returns contents of etLaserTriggerFilter as a double
hFpga = gethFpga();
hFpga.LinScanLaserTriggerFilterTicks = hObject.Value;

% --- Executes during object creation, after setting all properties.
function etLaserTriggerFilter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to etLaserTriggerFilter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function hFpga = gethFpga()
hSI = evalin('base','hSI');
vals = hSI.fpgaMap.values;
val = vals{1};
hFpga = val.hFpga;




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
