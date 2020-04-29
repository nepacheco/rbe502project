function varargout = gui(varargin)
% GUI MATLAB code for gui.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui

% Last Modified by GUIDE v2.5 04-Feb-2020 12:38:26

% Begin initialization code - DO NOT EDIT
gui_Singleton = 0;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_OutputFcn, ...
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


% --- Executes just before gui is made visible.
function gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui (see VARARGIN)

% Choose default command line output for gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
rotate3d on;
in = varargin(1);
global x y z curr_pos robot plot_on height
robot = in{1};
robot.p.Parent = handles.axes3;
 height = 0;
    for i=1:length(robot.c)
        height=height+robot.c(i);
    end
    for i=1:length(robot.h)
        height=height+robot.h(i);
    end
robot.run_ikin([0;0;0],[0;0;height],true)
x = 0;
y = 0;
z = height;
curr_pos = [0;0;0];
update_gui(handles);
set(handles.checkbox1, 'Value', 1);
plot_on = get(handles.checkbox1, 'Value');
set(handles.text22, 'String', height);
set(handles.text25, 'String', height + 5);
set(handles.slider5, 'Max', height + 5);
set(handles.slider5, 'Min', height);
set(handles.slider5, 'Value', height);
keyboard_control(robot, handles);
% UIWAIT makes gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function update_gui(handles)
    global x y z curr_pos robot plot_on
    
    if plot_on
        hold on
        axes(robot.p.Parent);
        children = robot.p.Parent.Children();
                j = length(children);
                for i=1:j
                    if isa(children(i), 'matlab.graphics.chart.primitive.Scatter')
                         j = j - 1;
                        delete(children(i));
                    end
                end 
        scatter3(x,y,z,'r*');
        hold off
    end
    
    set(handles.text29, 'String', x);
    set(handles.text32, 'String', y);
    set(handles.text33, 'String', z);
    temp = robot.run_ikin([curr_pos(1);curr_pos(2);curr_pos(3)],[x;y;z],false);
    curr_pos = temp;
    if plot_on
        robot.plot_robot(curr_pos(1),curr_pos(2),curr_pos(3));
    end
    set(handles.text15, 'String', curr_pos(1));
    set(handles.text18, 'String', curr_pos(2));
    set(handles.text19, 'String', curr_pos(3));
   

   
% --- Outputs from this function are returned to the command line.
function varargout = gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x y z

x = get(handles.slider1,'value');
y = get(handles.slider4,'value');
z = get(handles.slider5,'value');
update_gui(handles);
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)

% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x y z 

x = get(handles.slider1,'value');
y = get(handles.slider4,'value');
z = get(handles.slider5,'value');
update_gui(handles);


% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x y z 

x = get(handles.slider1,'value');
y = get(handles.slider4,'value');
z = get(handles.slider5,'value');
update_gui(handles);


% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

   
% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
   global plot_on
   plot_on = get(hObject, 'Value');
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
global x y z height
x = 0;
y = 0;
z = height;
set(handles.slider1,'value',x);
set(handles.slider4,'value',y);
set(handles.slider5,'value',z);
update_gui(handles);
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
