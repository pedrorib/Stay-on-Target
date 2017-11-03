function varargout = MicroPositioningSystem(varargin)
% MICROPOSITIONINGSYSTEM MATLAB code for MicroPositioningSystem.fig
%      MICROPOSITIONINGSYSTEM, by itself, creates a new MICROPOSITIONINGSYSTEM or raises the existing
%      singleton*.
%
%      H = MICROPOSITIONINGSYSTEM returns the handle to a new MICROPOSITIONINGSYSTEM or the handle to
%      the existing singleton*.
%
%      MICROPOSITIONINGSYSTEM('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MICROPOSITIONINGSYSTEM.M with the given input arguments.
%
%      MICROPOSITIONINGSYSTEM('Property','Value',...) creates a new MICROPOSITIONINGSYSTEM or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MicroPositioningSystem_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MicroPositioningSystem_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MicroPositioningSystem

% Last Modified by GUIDE v2.5 28-Oct-2017 21:51:13

% Begin initialization code - DO NOT EDIT
clear mypi mycam dat1
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @MicroPositioningSystem_OpeningFcn, ...
                   'gui_OutputFcn',  @MicroPositioningSystem_OutputFcn, ...
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

% --- Executes just before MicroPositioningSystem is made visible.
function MicroPositioningSystem_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MicroPositioningSystem (see VARARGIN)

% Choose default command line output for MicroPositioningSystem
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

global mypi mycam
global piON camON mycam_on flagz
global restart_ST change_sensor
restart_ST=false;
change_sensor=true;
mypi = 0;
mycam = 0;
piON = 0;
camON = 0;
mycam_on = true;
flagz = false;

no_video_symbol = imread('video_symbol.png');
axes(handles.axes_live_cam);
imshow(no_video_symbol);
axis(handles.axes_sensor_tile,[0 200 -Inf Inf]);

handles.h1 = animatedline(handles.axes_sensor_tile,'Color', 'Red');
handles.h2 = animatedline(handles.axes_sensor_tile,'Color', 'Green');
handles.h3 = animatedline(handles.axes_sensor_tile,'Color', 'Blue');
guidata(hObject, handles);

%up_img = imread('up.jpg');
%set(handles.pushbutton_pY,'CData',up_img)

%set(handles.axes_sensor_tile,'Visible','Off')


%st_logo = imread('plot.png');
%axes(handles.axes_sensor_tile);
%imshow(st_logo);

% --- Outputs from this function are returned to the command line.
function varargout = MicroPositioningSystem_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;


% --- Executes on button press in pushbutton_Connect.
function pushbutton_Connect_Callback(hObject, eventdata, handles)
global mypi mycam
global piON camON
global rotation_angle

mypi = 0;
mycam = 0;

delay = 1;
ip = get(handles.rasp_ip,'String');
password = get(handles.rasp_pw,'String');
username = get(handles.rasp_username,'String');
mac = get(handles.st_mac,'String');

%connect raspberry pi
[mypi, piON] = connect_rasp(ip,username,password, delay);

configurePin(mypi,5,'DigitalOutput');
writeDigitalPin(mypi,5,0);
%configurePin(mypi,19,'DigitalOutput');
%writeDigitalPin(mypi,19,1);

% connect camera default values
if piON == 1
    resolution = '640x480';
    quality = 10;
    rotation_angle = 180;
    rate = 30;
    brightness = 50;
    contrast = 0;
    saturation = 0;
    sharpness = 0;
    [~, camON] = connect_cam(mypi, resolution, quality, rotation_angle, rate, ...
        brightness, contrast, saturation, sharpness, delay);
    mycam = cameraboard(mypi,'Resolution',resolution,...
        'Quality',quality,...
        'Rotation',rotation_angle,...
        'FrameRate',rate,...
        'Brightness',brightness,...
        'Contrast',contrast,...
        'Saturation',saturation,...
        'Sharpness',sharpness);
end

% connect sensor tile
if camON == 1
   status_box = msgbox('Connecting Sensor Tile ...', 'Connecting');
   stON = 1;
   MAC = get(handles.st_mac,'String');
   handles.dat1 = sensor2(mypi,MAC);
   disp('DONE');
   handles.dat1.update('agm');
   handles.dat1.acc
   handles.dat1.gyr
   guidata(hObject,handles);
   delete(status_box)
   iconON = imread('checkmark.png');
   status_box = msgbox('Sensor Tile Connected','Success','custom',iconON);
   pause(delay)
   delete(status_box)
else
   stON = 0;
end

%initialize motors
if stON == 1
    mypi.AvailableDigitalPins
    global dir_x dir_y dir_z dir_theta dir_depth
    global motor_x motor_y motor_z motor_depth motor_theta Rmd_step
    Rmm_step = 0.5/513;     %[mm/step]
    Rmd_step = 545;    %[mm/step]
    Rdeg_step = 360/513;    %[deg/step]
    delay_max = 0.00001;    %[delay_max]
    delay_min = 0.00001;       %[delay_min]
    x_sweep = 10;           %[mm]
    y_sweep = 10;           %[mm]
    z_sweep = 10;           %[mm]
    theta_sweep = 60;
    depth_sweep = 1;
    %direcção decrescente
    dir_x = 0;
    dir_y = 0;
    dir_z = 0;
    dir_theta = -1;
    dir_depth = 0;
    freq = 200;
    
    a=0;
    k=0;
    for i=1:25
        handles.dat1.update('agm');
        if handles.dat1.sound > 40 && handles.dat1.sound < 100
            a = a + handles.dat1.sound;
            k=k+1;
        end
    end
    a=a/k;
    set(handles.show_edt,'String',num2str(int16(a)));
    handles.dat1.update();
    set(handles.show_hum,'String',num2str(handles.dat1.hum/10));
    set(handles.show_temp,'String',num2str(handles.dat1.temp1/10));
    
    status_box = msgbox('5D Stage Initialization ...', 'Motor Initialization');
    
    %config_motor(mypi,enbl_pin, step_pin, dir_pin, check_pin, resolution, sweep, rel_pos, dir, delay_min)
    %motor_depth = config_motor_PWM(mypi, 14, 15, 18, 22, Rmm_step, depth_sweep, 0, dir_depth, delay_min, delay_max, freq);
    motor_depth = config_motor_PWM(mypi, 14, 15, 18, 22, Rmm_step, depth_sweep, 0, dir_depth, delay_min, delay_max, freq);
    motor_z = config_motor_PWM(mypi, 14, 25, 12, 17, Rmm_step, z_sweep, 0, dir_z, delay_min, delay_max, freq);
    motor_theta = config_motor(mypi, 14, 21, 26, 17, Rdeg_step, theta_sweep, 0, -1, delay_min*100, delay_max);
    motor_x = config_motor_PWM(mypi,14, 23, 24, 27, Rmm_step, x_sweep, 0, dir_x, delay_min, delay_max, freq);
    motor_y = config_motor_PWM(mypi, 14, 16, 20, 13, Rmm_step, y_sweep, 0, dir_y, delay_min, delay_max, freq);
    
    %motor = [enbl_pin step_pin dir_pin check_pin resolution sweep delay_min delay_max];
    
    e_theta = 5;
    Res = Rdeg_step;
    theta_f = 0;
    %%% read value and plot acelerometer and angle %%%
    handles.dat1.update('agm');
    theta_current = atand(handles.dat1.acc(2)/handles.dat1.acc(3));
    
    writeDigitalPin(mypi,motor_theta(1),0);
    while abs(theta_current - theta_f) > e_theta
        if get(handles.pushbutton_pX,'Value') == 1
            break;
        end
        clearpoints(handles.h1);
        theta_i = theta_current;
        delta = theta_current - theta_f;
        Nstep = int16(delta / Res);
        %[end_move, N] = move_motor(mypi, motor_theta, abs(NStep), gt(Nstep,0), vel);        
        %%% function move %%%
        writeDigitalPin(mypi,motor_theta(3),gt(Nstep,0));
%         for i = 1:abs(Nstep)
%             writeDigitalPin(mypi,motor_theta(2),0);
%             pause(delay_min);
%             writeDigitalPin(mypi,motor_theta(2),1);
%             pause(delay_min);
%         end
        
       % p=gcp();
        %ftheta = parfeval(p,@init_theta,1,mypi,motor_theta,delay_min, abs(Nstep));
        %i=0;
        %while isempty(fetchNext(ftheta,0))
        for i = 1:abs(Nstep)
            writeDigitalPin(mypi,motor_theta(2),0);
            pause(delay_min);
            writeDigitalPin(mypi,motor_theta(2),1);
            pause(delay_min);
        end
            handles.dat1.update('agm');
            axis(handles.axes_sensor_tile,[0 200 -Inf Inf]);
            addpoints(handles.h1,double(i),double(atand(handles.dat1.acc(2)/handles.dat1.acc(3))));
            drawnow;
            i=i+1;
        %end
        %writeDigitalPin(mypi,motor_theta(1),1);
        %%% end function %%%
        %cancel(ftheta);
        pause(1);
        
        %%% read value %%%
        handles.dat1.update('agm');
        handles.dat1.update('agm');
        handles.dat1.update('agm');
        handles.dat1.update('agm');
        handles.dat1.update('agm');
        handles.dat1.update('agm');
        theta_current = atand(handles.dat1.acc(2)/handles.dat1.acc(3));
        
        Res = abs(theta_current - theta_i)/i;
    end
    
    pause(1);
    configurePin(mypi,5,'DigitalOutput');
    writeDigitalPin(mypi,5,1);
    configurePin(mypi,19,'DigitalOutput');
    writeDigitalPin(mypi,19,0);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%     clearpoints(handles.h1);
%     clearpoints(handles.h2);
%     clearpoints(handles.h3);
%     
%     a=[0:Rmd_step:1.5;0:Rmd_step:1.5;0:Rmd_step:1.5];
%     a=a';
%     
%     writeDigitalPin(mypi,motor_depth(1),0);
%     writeDigitalPin(mypi,motor_depth(3),not(dir_depth));
%     t=0;
%     axis(handles.axes_sensor_tile,[0 1 -Inf Inf]);
%     for i=0:Rmd_step:1.5
%         t=t+1;
%         writeDigitalPin(mypi,motor_depth(2),0);
%         pause(delay_min);
%         writeDigitalPin(mypi,motor_depth(2),1);
%         pause(delay_min);
%         axis(handles.axes_sensor_tile,[0 t -Inf Inf]);
%         handles.dat1.update('agm');
%         a(t,:)=handles.dat1.mag;
%         %addpoints(handles.h1,i,handles.dat1.mag(1));
%         %addpoints(handles.h2,i,handles.dat1.mag(2));
%         addpoints(handles.h3,t,handles.dat1.mag(3));
%         drawnow;
%     end
%   
% %   Determinar passo da rosca    
%     assignin('base','a_raw',a(:,3));
%     a(:,3)=smooth(a(:,3),100);
%     %a(:,3)=detrend(a(:,3));
%     clearpoints(handles.h1);
%     clearpoints(handles.h2);
%     clearpoints(handles.h3);
%     for i=1:length(a(:,3))
%         addpoints(handles.h3,i,a(i,3));
%     end
% 
%     [~,Midx]=findpeaks(a(:,3),'MinPeakDistance',200);
% %    [~,midx]=findpeaks(1.01*max(a(:,3))-a(:,3),'MinPeakDistance',200);
%     
%     Rmd_step = double(abs(Midx(2)-Midx(4)));
%         
%     writeDigitalPin(mypi,motor_depth(3),dir_depth);
% 
%     b=t:-1:Midx(4);
%     
%     for i=t:-1:Midx(4)
%         writeDigitalPin(mypi,motor_depth(2),0);
%         pause(delay_min);
%         writeDigitalPin(mypi,motor_depth(2),1);
%         pause(delay_min);
%         handles.dat1.update('agm');
%         addpoints(handles.h2,i,handles.dat1.mag(3));
%         b(t-i+1)=handles.dat1.mag(3);
%     end
%     
%     
    
%     assignin('base','Midx',Midx);
%     assignin('base','b_raw',b);
%     assignin('base','Rmd_step',Rmd_step);
%     
%     
    
%     pause(5);
    clearpoints(handles.h3);

%%%%%%%%%%%%
    %motor_depth = config_motor(mypi, 4, 22, 23, 24, Rmm_step, depth_sweep, 0, dir_depth, delay_min, delay_max);
    %calibrate depth motor
    
    
    mymotor = 1;
else 
    mymotor = 0;
end

delete(status_box)
iconON = imread('checkmark.png');
status_box = msgbox('Initialization Completed','Success','custom',iconON);
pause(1)
delete(status_box)

function rasp_ip_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function rasp_ip_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rasp_ip (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function st_mac_Callback(hObject, eventdata, handles)
% hObject    handle to st_mac (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of st_mac as text
%        str2double(get(hObject,'String')) returns contents of st_mac as a double

% --- Executes during object creation, after setting all properties.
function st_mac_CreateFcn(hObject, eventdata, handles)
% hObject    handle to st_mac (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in togglebutton_acquire.
function togglebutton_acquire_Callback(hObject, eventdata, handles)
global mypi mycam
global camON live change_sensor restart_ST
global warning i j
global rotation_angle 
mycam = 0;

if camON == 1
    buttonON = get(handles.togglebutton_acquire,'Value');
    if buttonON == get(handles.togglebutton_acquire,'Max')
        rate = int8(str2double(get(handles.frame_rate,'String')));
        if rate >= 2 && rate <= 30

            set(handles.togglebutton_acquire,'String','Stop')

            %get values - convert values - set values
            resolution_menu = cellstr(get(handles.popupmenu_resolution,'String'));
            resolution = resolution_menu{get(handles.popupmenu_resolution,'Value')};

            quality100 = get(handles.slider_quality,'Value');
            quality = int8(1 + (100-1) * quality100);
            %set(handles.edit_quality,'String', num2str(quality));

            brightness100 = get(handles.slider_brightness,'Value');
            brightness = int8(0 + (100-0) * brightness100);
            %set(handles.edit_brightness,'String', num2str(brightness));

            contrast100 = get(handles.slider_contrast,'Value');
            contrast = int8(-100 + (100+100) * contrast100);
            %set(handles.edit_contrast,'String', num2str(contrast));

            saturation100 = get(handles.slider_saturation,'Value');
            saturation = int8(-100 + (100+100) * saturation100);
            %set(handles.edit_saturation,'String', num2str(saturation));

            sharpness100 = get(handles.slider_sharpness,'Value');
            sharpness = int8(-100 + (100+100) * sharpness100);
            %set(handles.edit_sharpness,'String', num2str(sharpness));       

            %configure camera
            
            mycam = cameraboard(mypi,'Resolution',resolution,...
                'Quality',quality,...
                'Rotation',rotation_angle,...
                'FrameRate',rate,...
                'Brightness',brightness,...
                'Contrast',contrast,...
                'Saturation',saturation,...
                'Sharpness',sharpness);
            
            %show
            live = 1;
            %axes(handles.axes_live_cam);
            while mycam ~= 0
                [i,change_sensor]=show_sensor(change_sensor, handles, restart_ST,i);
                if j == 0
                    i=j;
                    j=j+1;
                end
                %show_cam(handles,mycam, live);
                if live == 1
                    img = snapshot(mycam);
                    imshow(img, 'Parent', handles.axes_live_cam);
                    drawnow;
                end
            end
        else
            msgbox('FrameRate Value must be between [2,30] fps','Error','error');
            set(handles.frame_rate,'String','30');
            set(handles.togglebutton_acquire,'Value', 0);
            set(handles.togglebutton_acquire,'String','Acquire');
            live = 0;

        end
    elseif buttonON == get(handles.togglebutton_acquire,'Min')
        set(handles.togglebutton_acquire,'String','Acquire')
%         mycam = 0;
%         clear mycam;
        live = 0;
        warning = 0;
    end
else
    set(handles.togglebutton_acquire,'Value',0);
    msgbox('Camera not initialized','Error','error');
    live = 0;
end
 




% --- Executes on selection change in popupmenu_resolution.
function popupmenu_resolution_Callback(hObject, eventdata, handles)
global mypi mycam
global camON live
global warning
global rotation_angle
mycam = 0;
warning = 1;

rate = int8(str2double(get(handles.frame_rate,'String')));

resolution_menu = cellstr(get(handles.popupmenu_resolution,'String'));
resolution = resolution_menu{get(handles.popupmenu_resolution,'Value')};

quality100 = get(handles.slider_quality,'Value');
quality = int8(1 + (100-1) * quality100);
%set(handles.edit_quality,'String', num2str(quality));

brightness100 = get(handles.slider_brightness,'Value');
brightness = int8(0 + (100-0) * brightness100);
%set(handles.edit_brightness,'String', num2str(brightness));

contrast100 = get(handles.slider_contrast,'Value');
contrast = int8(-100 + (100+100) * contrast100);
%set(handles.edit_contrast,'String', num2str(contrast));

saturation100 = get(handles.slider_saturation,'Value');
saturation = int8(-100 + (100+100) * saturation100);
%set(handles.edit_saturation,'String', num2str(saturation));

sharpness100 = get(handles.slider_sharpness,'Value');
sharpness = int8(-100 + (100+100) * sharpness100);
%set(handles.edit_sharpness,'String', num2str(sharpness));       
           
if camON == 1 && live == 1
    set(handles.popupmenu_resolution,'Enable','Off')
%     try
        %configure camera
        mycam = cameraboard(mypi,'Resolution',resolution,...
                                 'Quality',quality,...
                                 'Rotation',rotation_angle,...
                                 'FrameRate',rate,...
                                 'Brightness',brightness,...
                                 'Contrast',contrast,...
                                 'Saturation',saturation,...
                                  'Sharpness',sharpness);
%     catch
%         mycam = 0;
%         live = 0;
%     end
%     set(handles.popupmenu_resolution,'Enable','On')
%     axes(handles.axes_live_cam);
%     while live == 1 && mycam~= 0
%         try         
%             img = snapshot(mycam);
%             imshow(img);
%             drawnow;
%         catch
%             live = 0;
%             mycam = 0;
%         end 
%     end

    if live == 0 && mycam == 0 && warning == 1
        msgbox('Camera Snapshot Failed - Press Acquire Again','Error','error');
        set(handles.togglebutton_acquire,'Value', 0);
        set(handles.togglebutton_acquire,'String','Acquire');
        warning = 0;
    end
end


% --- Executes during object creation, after setting all properties.
function popupmenu_resolution_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function frame_rate_Callback(hObject, eventdata, handles)
global mypi mycam
global camON live
global warning
global rotation_angle
    
rate = int8(str2double(get(handles.frame_rate,'String')));

resolution_menu = cellstr(get(handles.popupmenu_resolution,'String'));
resolution = resolution_menu{get(handles.popupmenu_resolution,'Value')};

quality100 = get(handles.slider_quality,'Value');
quality = int8(1 + (100-1) * quality100);
%set(handles.edit_quality,'String', num2str(quality));

brightness100 = get(handles.slider_brightness,'Value');
brightness = int8(0 + (100-0) * brightness100);
%set(handles.edit_brightness,'String', num2str(brightness));

contrast100 = get(handles.slider_contrast,'Value');
contrast = int8(-100 + (100+100) * contrast100);
%set(handles.edit_contrast,'String', num2str(contrast));

saturation100 = get(handles.slider_saturation,'Value');
saturation = int8(-100 + (100+100) * saturation100);
%set(handles.edit_saturation,'String', num2str(saturation));

sharpness100 = get(handles.slider_sharpness,'Value');
sharpness = int8(-100 + (100+100) * sharpness100);
%set(handles.edit_sharpness,'String', num2str(sharpness)); 

if rate >= 2 && rate <= 30
    mycam = 0;
    warning = 1;
    if camON == 1 && live == 1
        set(handles.frame_rate,'Enable','Off')
        try
            mycam = cameraboard(mypi,'Resolution',resolution,...
                                     'Quality',quality,...
                                     'Rotation',rotation_angle,...
                                     'FrameRate',rate,...
                                     'Brightness',brightness,...
                                     'Contrast',contrast,...
                                     'Saturation',saturation,...
                                     'Sharpness',sharpness);
        catch
            mycam = 0;
            live = 0;
        end
        set(handles.frame_rate,'Enable','On')
        axes(handles.axes_live_cam);
        while live == 1 && mycam ~=0
            try         
                img = snapshot(mycam);
                imshow(img);
                drawnow;
            catch
                live = 0;
                mycam = 0;
            end 
        end
        
        if live == 0 && mycam == 0 && warning == 1
            msgbox('Camera Snapshot Failed - Press Acquire Again','Error','error');
            set(handles.togglebutton_acquire,'Value', 0);
            set(handles.togglebutton_acquire,'String','Acquire');
            warning = 0;
        end  
    end
else
    msgbox('FrameRate Value must be between [2,30] fps','Error','error');
    set(handles.frame_rate,'String','30');
    set(handles.togglebutton_acquire,'Value', 0);
    set(handles.togglebutton_acquire,'String','Acquire');
    live = 0;
end





% --- Executes during object creation, after setting all properties.
function frame_rate_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton_snap.
function pushbutton_snap_Callback(hObject, eventdata, handles)
global camON live

if camON == 1 && live == 1
    imsave(handles.axes_live_cam);
else
    msgbox('Camera Off','Error','error');
end



function rasp_username_Callback(hObject, eventdata, handles)
% hObject    handle to rasp_username (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rasp_username as text
%        str2double(get(hObject,'String')) returns contents of rasp_username as a double


% --- Executes during object creation, after setting all properties.
function rasp_username_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rasp_username (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function rasp_pw_Callback(hObject, eventdata, handles)
% hObject    handle to rasp_pw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rasp_pw as text
%        str2double(get(hObject,'String')) returns contents of rasp_pw as a double


% --- Executes during object creation, after setting all properties.
function rasp_pw_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rasp_pw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_brightness_Callback(hObject, eventdata, handles)
global mypi mycam
global camON live
global warning
global rotation_angle
mycam = 0;
warning = 1;

rate = int8(str2double(get(handles.frame_rate,'String')));

resolution_menu = cellstr(get(handles.popupmenu_resolution,'String'));
resolution = resolution_menu{get(handles.popupmenu_resolution,'Value')};

quality100 = get(handles.slider_quality,'Value');
quality = int8(1 + (100-1) * quality100);
%set(handles.edit_quality,'String', num2str(quality));

brightness100 = get(handles.slider_brightness,'Value');
brightness = int8(0 + (100-0) * brightness100);
set(handles.edit_brightness,'String', num2str(brightness));

contrast100 = get(handles.slider_contrast,'Value');
contrast = int8(-100 + (100+100) * contrast100);
%set(handles.edit_contrast,'String', num2str(contrast));

saturation100 = get(handles.slider_saturation,'Value');
saturation = int8(-100 + (100+100) * saturation100);
%set(handles.edit_saturation,'String', num2str(saturation));

sharpness100 = get(handles.slider_sharpness,'Value');
sharpness = int8(-100 + (100+100) * sharpness100);
%set(handles.edit_sharpness,'String', num2str(sharpness)); 

if camON == 1 && live == 1
    set(handles.slider_brightness,'Enable','Off')
    try
        mycam = cameraboard(mypi,'Resolution',resolution,...
                                 'Quality',quality,...
                                 'Rotation',rotation_angle,...
                                 'FrameRate',rate,...
                                 'Brightness',brightness,...
                                 'Contrast',contrast,...
                                 'Saturation',saturation,...
                                 'Sharpness',sharpness);
    catch
        mycam = 0;
        live = 0;
    end
    set(handles.slider_brightness,'Enable','On')
%     axes(handles.axes_live_cam);
%     while live == 1 && mycam~= 0
%         try         
%             img = snapshot(mycam);
%             imshow(img);
%             drawnow;
%         catch
%             live = 0;
%             mycam = 0;
%         end 
%     end
% 
%     if live == 0 && mycam == 0 && warning == 1
%         msgbox('Camera Snapshot Failed - Press Acquire Again','Error','error');
%         set(handles.togglebutton_acquire,'Value', 0);
%         set(handles.togglebutton_acquire,'String','Acquire');
%         warning = 0;
%     end
end



% --- Executes during object creation, after setting all properties.
function slider_brightness_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_contrast_Callback(hObject, eventdata, handles)
global mypi mycam
global camON live
global warning
global rotation_angle
mycam = 0;
warning = 1;
    
rate = int8(str2double(get(handles.frame_rate,'String')));

resolution_menu = cellstr(get(handles.popupmenu_resolution,'String'));
resolution = resolution_menu{get(handles.popupmenu_resolution,'Value')};

quality100 = get(handles.slider_quality,'Value');
quality = int8(1 + (100-1) * quality100);
%set(handles.edit_quality,'String', num2str(quality));

brightness100 = get(handles.slider_brightness,'Value');
brightness = int8(0 + (100-0) * brightness100);
%set(handles.edit_brightness,'String', num2str(brightness));

contrast100 = get(handles.slider_contrast,'Value');
contrast = int8(-100 + (100+100) * contrast100);
set(handles.edit_contrast,'String', num2str(contrast));

saturation100 = get(handles.slider_saturation,'Value');
saturation = int8(-100 + (100+100) * saturation100);
%set(handles.edit_saturation,'String', num2str(saturation));

sharpness100 = get(handles.slider_sharpness,'Value');
sharpness = int8(-100 + (100+100) * sharpness100);
%set(handles.edit_sharpness,'String', num2str(sharpness)); 


if camON == 1 && live == 1
    set(handles.slider_contrast,'Enable','Off')
    try
        mycam = cameraboard(mypi,'Resolution',resolution,...
                                 'Quality',quality,...
                                 'Rotation',rotation_angle,...
                                 'FrameRate',rate,...
                                 'Brightness',brightness,...
                                 'Contrast',contrast,...
                                 'Saturation',saturation,...
                                 'Sharpness',sharpness);
    catch
        mycam = 0;
        live = 0;
    end
    set(handles.slider_contrast,'Enable','On')
%     axes(handles.axes_live_cam);
%     while live == 1 && mycam ~=0
%         try         
%             img = snapshot(mycam);
%             imshow(img);
%             drawnow;
%         catch
%             live = 0;
%             mycam = 0;
%         end 
%     end
% 
%     if live == 0 && mycam == 0 && warning == 1
%         msgbox('Camera Snapshot Failed - Press Acquire Again','Error','error');
%         set(handles.togglebutton_acquire,'Value', 0);
%         set(handles.togglebutton_acquire,'String','Acquire');
%         warning = 0;
%     end
end



% --- Executes during object creation, after setting all properties.
function slider_contrast_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_saturation_Callback(hObject, eventdata, handles)
global mypi mycam
global camON live
global warning
global rotation_angle
mycam = 0;
warning = 1;
    
rate = int8(str2double(get(handles.frame_rate,'String')));

resolution_menu = cellstr(get(handles.popupmenu_resolution,'String'));
resolution = resolution_menu{get(handles.popupmenu_resolution,'Value')};

quality100 = get(handles.slider_quality,'Value');
quality = int8(1 + (100-1) * quality100);
%set(handles.edit_quality,'String', num2str(quality));

brightness100 = get(handles.slider_brightness,'Value');
brightness = int8(0 + (100-0) * brightness100);
%set(handles.edit_brightness,'String', num2str(brightness));

contrast100 = get(handles.slider_contrast,'Value');
contrast = int8(-100 + (100+100) * contrast100);
%set(handles.edit_contrast,'String', num2str(contrast));

saturation100 = get(handles.slider_saturation,'Value');
saturation = int8(-100 + (100+100) * saturation100);
set(handles.edit_saturation,'String', num2str(saturation));

sharpness100 = get(handles.slider_sharpness,'Value');
sharpness = int8(-100 + (100+100) * sharpness100);
%set(handles.edit_sharpness,'String', num2str(sharpness)); 

if camON == 1 && live == 1
    set(handles.slider_saturation,'Enable','Off')
    try
        mycam = cameraboard(mypi,'Resolution',resolution,...
                                 'Quality',quality,...
                                 'Rotation',rotation_angle,...
                                 'FrameRate',rate,...
                                 'Brightness',brightness,...
                                 'Contrast',contrast,...
                                 'Saturation',saturation,...
                                 'Sharpness',sharpness);
    catch
        mycam = 0;
        live = 0;
    end
    set(handles.slider_saturation,'Enable','On')
%     axes(handles.axes_live_cam);
%     while live == 1 && mycam ~=0
%         try         
%             img = snapshot(mycam);
%             imshow(img);
%             drawnow;
%         catch
%             live = 0;
%             mycam = 0;
%         end 
%     end
% 
%     if live == 0 && mycam == 0 && warning == 1
%         msgbox('Camera Snapshot Failed - Press Acquire Again','Error','error');
%         set(handles.togglebutton_acquire,'Value', 0);
%         set(handles.togglebutton_acquire,'String','Acquire');
%         warning = 0;
%     end
end


% --- Executes during object creation, after setting all properties.
function slider_saturation_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_sharpness_Callback(hObject, eventdata, handles)
global mypi mycam
global camON live
global warning
global rotation_angle
mycam = 0;
warning = 1;
 
rate = int8(str2double(get(handles.frame_rate,'String')));

resolution_menu = cellstr(get(handles.popupmenu_resolution,'String'));
resolution = resolution_menu{get(handles.popupmenu_resolution,'Value')};

quality100 = get(handles.slider_quality,'Value');
quality = int8(1 + (100-1) * quality100);
%set(handles.edit_quality,'String', num2str(quality));

brightness100 = get(handles.slider_brightness,'Value');
brightness = int8(0 + (100-0) * brightness100);
%set(handles.edit_brightness,'String', num2str(brightness));

contrast100 = get(handles.slider_contrast,'Value');
contrast = int8(-100 + (100+100) * contrast100);
%set(handles.edit_contrast,'String', num2str(contrast));

saturation100 = get(handles.slider_saturation,'Value');
saturation = int8(-100 + (100+100) * saturation100);
%set(handles.edit_saturation,'String', num2str(saturation));

sharpness100 = get(handles.slider_sharpness,'Value');
sharpness = int8(-100 + (100+100) * sharpness100);
set(handles.edit_sharpness,'String', num2str(sharpness)); 

if camON == 1 && live == 1
    set(handles.slider_sharpness,'Enable','Off')
    try
        mycam = cameraboard(mypi,'Resolution',resolution,...
                                 'Quality',quality,...
                                 'Rotation',rotation_angle,...
                                 'FrameRate',rate,...
                                 'Brightness',brightness,...
                                 'Contrast',contrast,...
                                 'Saturation',saturation,...
                                 'Sharpness',sharpness);
    catch
        mycam = 0;
        live = 0;
    end
    set(handles.slider_sharpness,'Enable','On')
%     axes(handles.axes_live_cam);
%     while live == 1 && mycam ~=0
%         try         
%             img = snapshot(mycam);
%             imshow(img);
%             drawnow;
%         catch
%             live = 0;
%             mycam = 0;
%         end 
%     end
% 
%     if live == 0 && mycam == 0 && warning == 1
%         msgbox('Camera Snapshot Failed - Press Acquire Again','Error','error');
%         set(handles.togglebutton_acquire,'Value', 0);
%         set(handles.togglebutton_acquire,'String','Acquire');
%         warning = 0;
%     end
end



% --- Executes during object creation, after setting all properties.
function slider_sharpness_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_quality_Callback(hObject, eventdata, handles)
global mypi mycam
global camON live
global warning
global rotation_angle
mycam = 0;
warning = 1;

rate = int8(str2double(get(handles.frame_rate,'String')));

resolution_menu = cellstr(get(handles.popupmenu_resolution,'String'));
resolution = resolution_menu{get(handles.popupmenu_resolution,'Value')};

quality100 = get(handles.slider_quality,'Value');
quality = int8(1 + (100-1) * quality100);
set(handles.edit_quality,'String', num2str(quality));

brightness100 = get(handles.slider_brightness,'Value');
brightness = int8(0 + (100-0) * brightness100);
%set(handles.edit_brightness,'String', num2str(brightness));

contrast100 = get(handles.slider_contrast,'Value');
contrast = int8(-100 + (100+100) * contrast100);
%set(handles.edit_contrast,'String', num2str(contrast));

saturation100 = get(handles.slider_saturation,'Value');
saturation = int8(-100 + (100+100) * saturation100);
%set(handles.edit_saturation,'String', num2str(saturation));

sharpness100 = get(handles.slider_sharpness,'Value');
sharpness = int8(-100 + (100+100) * sharpness100);
%set(handles.edit_sharpness,'String', num2str(sharpness)); 

if camON == 1 && live == 1
    set(handles.slider_quality,'Enable','Off')
    try
        mycam = cameraboard(mypi,'Resolution',resolution,...
                                 'Quality',quality,...
                                 'Rotation',rotation_angle,...
                                 'FrameRate',rate,...
                                 'Brightness',brightness,...
                                 'Contrast',contrast,...
                                 'Saturation',saturation,...
                                 'Sharpness',sharpness);
    catch
        mycam = 0;
        live = 0;
    end
    set(handles.slider_quality,'Enable','On')
%     %axes(handles.axes_live_cam);
%     while mycam ~=0
%         try         
%             show_cam(handles,mycam, live);
%             %i=show_sensor(change_sensor, handles, restart_ST,i);
%         catch
%             live = 0;
%             mycam = 0;
%         end 
%     end
% 
%     if live == 0 && mycam == 0 && warning == 1
%         msgbox('Camera Snapshot Failed - Press Acquire Again','Error','error');
%         set(handles.togglebutton_acquire,'Value', 0);
%         set(handles.togglebutton_acquire,'String','Acquire');
%         warning = 0;
%     end
end

% --- Executes during object creation, after setting all properties.
function slider_quality_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over text17.
function text17_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to text17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit_quality_Callback(hObject, eventdata, handles)
% hObject    handle to edit_quality (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_quality as text
%        str2double(get(hObject,'String')) returns contents of edit_quality as a double


% --- Executes during object creation, after setting all properties.
function edit_quality_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_quality (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_brightness_Callback(hObject, eventdata, handles)
% hObject    handle to edit_brightness (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_brightness as text
%        str2double(get(hObject,'String')) returns contents of edit_brightness as a double


% --- Executes during object creation, after setting all properties.
function edit_brightness_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_brightness (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_contrast_Callback(hObject, eventdata, handles)
% hObject    handle to edit_contrast (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_contrast as text
%        str2double(get(hObject,'String')) returns contents of edit_contrast as a double


% --- Executes during object creation, after setting all properties.
function edit_contrast_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_contrast (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_saturation_Callback(hObject, eventdata, handles)
% hObject    handle to edit_saturation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_saturation as text
%        str2double(get(hObject,'String')) returns contents of edit_saturation as a double


% --- Executes during object creation, after setting all properties.
function edit_saturation_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_saturation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_sharpness_Callback(hObject, eventdata, handles)
% hObject    handle to edit_sharpness (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_sharpness as text
%        str2double(get(hObject,'String')) returns contents of edit_sharpness as a double


% --- Executes during object creation, after setting all properties.
function edit_sharpness_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_sharpness (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2


% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_default_camera.
function pushbutton_default_camera_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_default_camera (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%set(handles.edit_quality,'String', num2str(quality));
global mypi mycam
global camON live
global warning
global rotation_angle
mycam = 0;
warning = 1;

rate = 30;
quality = 10;
brightness = 50;
contrast = 0;
saturation = 0;
sharpness = 0;

set(handles.frame_rate,'String',num2str(rate));

set(handles.popupmenu_resolution,'Value',3);
resolution_menu = cellstr(get(handles.popupmenu_resolution,'String'));
resolution = resolution_menu{3};

set(handles.edit_quality,'String', num2str(quality));
quality100 = (quality - 1) / (100 - 1);
set(handles.slider_quality,'Value', quality100);

set(handles.edit_brightness,'String', num2str(brightness));
brightness100 = (brightness - 0) / (100 - 0);
set(handles.slider_brightness,'Value', brightness100);

set(handles.edit_contrast,'String', num2str(contrast));
contrast100 = (contrast + 100) / (100 + 100);
set(handles.slider_contrast,'Value', contrast100);

set(handles.edit_saturation,'String', num2str(saturation));
saturation100 = (saturation + 100) / (100 + 100);
set(handles.slider_saturation,'Value', saturation100);

set(handles.edit_sharpness,'String', num2str(sharpness));
sharpness100 = (sharpness + 100) / (100 + 100);
set(handles.slider_sharpness,'Value', sharpness100);

if camON == 1 && live == 1
    set(handles.pushbutton_default_camera,'Enable','Off')
    try
        mycam = cameraboard(mypi,'Resolution',resolution,...
                                 'Quality',quality,...
                                 'Rotation',rotation_angle,...
                                 'FrameRate',rate,...
                                 'Brightness',brightness,...
                                 'Contrast',contrast,...
                                 'Saturation',saturation,...
                                 'Sharpness',sharpness);
    catch
        mycam = 0;
        live = 0;
    end
%     set(handles.pushbutton_default_camera,'Enable','On')
%     axes(handles.axes_live_cam);
%     while live == 1 && mycam~= 0
%         try         
%             img = snapshot(mycam);
%             imshow(img);
%             drawnow;
%         catch
%             live = 0;
%             mycam = 0;
%         end 
%     end
% 
%     if live == 0 && mycam == 0 && warning == 1
%         msgbox('Camera Snapshot Failed - Press Acquire Again','Error','error');
%         set(handles.togglebutton_acquire,'Value', 0);
%         set(handles.togglebutton_acquire,'String','Acquire');
%         warning = 0;
%     end
end


% --- Executes on selection change in popupmenu_ST.
function popupmenu_ST_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_ST (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_ST contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_ST
global change_sensor j;
change_sensor=true;
clearpoints(handles.h1);
clearpoints(handles.h2);
clearpoints(handles.h3);
j=0;

% --- Executes during object creation, after setting all properties.
function popupmenu_ST_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_ST (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pushbutton_acquire_ST.
function pushbutton_acquire_ST_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_acquire_ST (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global restart_ST change_sensor i mycam live j mycam_on
%axes_sensor_tile=handles.axes_sensor_tile;
%handles.h1 = animatedline('Color', 'Blue','Parent',handles.axes_sensor_tile);
%handles.h2 = animatedline('Color', 'Red','Parent',handles.axes_sensor_tile);
%handles.h3 = animatedline('Color', 'Green','Parent',handles.axes_sensor_tile);

%i=0;
%axes(handles.axes_sensor_tile);

restart_ST = get(hObject,'Value');

while get(hObject,'Value') == get(hObject,'Max') && mycam ~= 0
    set(handles.pushbutton_acquire_ST,'String','Stop');
    [i,change_sensor]=show_sensor(change_sensor, handles, restart_ST,i);
    if live == 1
        img = snapshot(mycam);
        imshow(img, 'Parent', handles.axes_live_cam);
        drawnow;
    end
    mycam_on = true;
    if j == 0
        i=j;
        j=j+1;
    end
%     end
%     sensor2 = get(handles.popupmenu_ST,'Value');
%     %sensor2=char(sensor2);
%     
%     if change_sensor == true
%         clearpoints(handles.h1);
%         clearpoints(handles.h2);
%         clearpoints(handles.h3);
%         i=0;
%         change_sensor = false;
%     end
%     switch sensor2
%         case 1
%             handles.dat1.update('agm');
%             if i<200
%                 addpoints(handles.h1,i,handles.dat1.acc(1))
%                 addpoints(handles.h2,i,handles.dat1.acc(2))
%                 addpoints(handles.h3,i,handles.dat1.acc(3))
%             else
%                 axis(handles.axes_sensor_tile,[i-200 i -Inf Inf])
%                 addpoints(handles.h1,i,handles.dat1.acc(1))
%                 addpoints(handles.h2,i,handles.dat1.acc(2))
%                 addpoints(handles.h3,i,handles.dat1.acc(3))
%             end
%         case 2
%             handles.dat1.update('agm');
%             if i<200
%                 addpoints(handles.h1,i,handles.dat1.gyr(1))
%                 addpoints(handles.h2,i,handles.dat1.gyr(2))
%                 addpoints(handles.h3,i,handles.dat1.gyr(3))
%             else
%                 axis(handles.axes_sensor_tile,[i-200 i -Inf Inf])
%                 addpoints(handles.h1,i,handles.dat1.gyr(1))
%                 addpoints(handles.h2,i,handles.dat1.gyr(2))
%                 addpoints(handles.h3,i,handles.dat1.gyr(3))
%             end
%         case 3
%             handles.dat1.update('agm');
%             if i<200
%                 addpoints(handles.h1,i,handles.dat1.mag(1))
%                 addpoints(handles.h2,i,handles.dat1.mag(2))
%                 addpoints(handles.h3,i,handles.dat1.mag(3))
%             else
%                 axis(handles.axes_sensor_tile,[i-200 i -Inf Inf])
%                 addpoints(handles.h1,i,handles.dat1.mag(1))
%                 addpoints(handles.h2,i,handles.dat1.mag(2))
%                 addpoints(handles.h3,i,handles.dat1.mag(3))
%             end
%         case 5
%             handles.dat1.update('pt');
%             if i<200
%                 addpoints(handles.h1,i,handles.dat1.temp);
%             else
%                 axis(handles.axes_sensor_tile,[i-200 i -Inf Inf])
%                 addpoints(handles.h1,i,handles.dat1.temp);
%             end
%         case 4
%             handles.dat1.update('pt');
%             if i<200
%                 addpoints(handles.h1,i,handles.dat1.pres);
%             else
%                 axis(handles.axes_sensor_tile,[i-200 i -Inf Inf])
%                 addpoints(handles.h1,i,handles.dat1.pres);
%             end
%     end
%     drawnow;
%     i=i+1;
end
while get(hObject,'Value') == get(hObject,'Min') && mycam ~= 0
        %show_cam(handles,mycam, live);
        set(handles.pushbutton_acquire_ST,'String','Acquire')
        if live == 1
            img = snapshot(mycam);
            imshow(img, 'Parent', handles.axes_live_cam);
            drawnow;
        end
        mycam_on = true;
        if j == 0
            i=j;
            j=j+1;
        end
end
while get(hObject,'Value') == get(hObject,'Max') && mycam == 0
    set(handles.pushbutton_acquire_ST,'String','Stop');
    [i,change_sensor]=show_sensor(change_sensor, handles, restart_ST,i);
    if j == 0
        i=j;
        j=j+1;
    end
%     end
%     sensor2 = get(handles.popupmenu_ST,'Value');
%     %sensor2=char(sensor2);
%     
%     if change_sensor == true
%         clearpoints(handles.h1);
%         clearpoints(handles.h2);
%         clearpoints(handles.h3);
%         i=0;
%         change_sensor = false;
%     end
%     switch sensor2
%         case 1
%             handles.dat1.update('agm');
%             if i<200
%                 addpoints(handles.h1,i,handles.dat1.acc(1))
%                 addpoints(handles.h2,i,handles.dat1.acc(2))
%                 addpoints(handles.h3,i,handles.dat1.acc(3))
%             else
%                 axis(handles.axes_sensor_tile,[i-200 i -Inf Inf])
%                 addpoints(handles.h1,i,handles.dat1.acc(1))
%                 addpoints(handles.h2,i,handles.dat1.acc(2))
%                 addpoints(handles.h3,i,handles.dat1.acc(3))
%             end
%         case 2
%             handles.dat1.update('agm');
%             if i<200
%                 addpoints(handles.h1,i,handles.dat1.gyr(1))
%                 addpoints(handles.h2,i,handles.dat1.gyr(2))
%                 addpoints(handles.h3,i,handles.dat1.gyr(3))
%             else
%                 axis(handles.axes_sensor_tile,[i-200 i -Inf Inf])
%                 addpoints(handles.h1,i,handles.dat1.gyr(1))
%                 addpoints(handles.h2,i,handles.dat1.gyr(2))
%                 addpoints(handles.h3,i,handles.dat1.gyr(3))
%             end
%         case 3
%             handles.dat1.update('agm');
%             if i<200
%                 addpoints(handles.h1,i,handles.dat1.mag(1))
%                 addpoints(handles.h2,i,handles.dat1.mag(2))
%                 addpoints(handles.h3,i,handles.dat1.mag(3))
%             else
%                 axis(handles.axes_sensor_tile,[i-200 i -Inf Inf])
%                 addpoints(handles.h1,i,handles.dat1.mag(1))
%                 addpoints(handles.h2,i,handles.dat1.mag(2))
%                 addpoints(handles.h3,i,handles.dat1.mag(3))
%             end
%         case 5
%             handles.dat1.update('pt');
%             if i<200
%                 addpoints(handles.h1,i,handles.dat1.temp);
%             else
%                 axis(handles.axes_sensor_tile,[i-200 i -Inf Inf])
%                 addpoints(handles.h1,i,handles.dat1.temp);
%             end
%         case 4
%             handles.dat1.update('pt');
%             if i<200
%                 addpoints(handles.h1,i,handles.dat1.pres);
%             else
%                 axis(handles.axes_sensor_tile,[i-200 i -Inf Inf])
%                 addpoints(handles.h1,i,handles.dat1.pres);
%             end
%     end
%     drawnow;
%     i=i+1;
end
    set(handles.pushbutton_acquire_ST,'String','Acquire');



%restart_ST = false;

% Hint: get(hObject,'Value') returns toggle state of pushbutton_acquire_ST


% --- Executes on button press in pushbutton_restart_ST.
function pushbutton_restart_ST_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_restart_ST (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global i j
i=0;
j=0;
clearpoints(handles.h1);
clearpoints(handles.h2);
clearpoints(handles.h3);




% --- Executes on button press in pushbutton_pY.
function pushbutton_pY_Callback(hObject, eventdata, handles)
global mypi
global dir_y
global motor_y
global mycam

motor = motor_y;
Dir = not(dir_y);
%motor = [enbl_pin step_pin dir_pin check_pin resolution sweep delay_min delay_max];

writeDigitalPin(mypi,motor(1),0);
writeDigitalPin(mypi,motor(3),Dir);

freq_menu = cellstr(get(handles.popupmenu_PWM,'String'));
freq = str2double(freq_menu{get(handles.popupmenu_PWM,'Value')});

%compute velocity
% velocity = get(handles.slider_velocity,'Value');
% set(handles.edit_velocity,'String', num2str(int8(velocity*100)));
% delay = (motor(7) - motor(8))/(1-0) * velocity + motor(8);

%get value
%initial_position = str2double(get(handles.edit_PositionY,'String'));

i=0;

configurePin(mypi, motor(2), 'PWM');
writePWMDutyCycle(mypi, motor(2), 0.5);
writePWMFrequency(mypi, motor(2), freq);

while get(hObject,'Value') == 1 
    %disp(rand)
    %pause(0.5)
%     if readDigitalPin(mypi,motor(4)) == 1
%         msgbox('Motor position out of range', 'Error','error');
%         break
%     end
%     writeDigitalPin(mypi,motor(2),0);
%     pause(delay);
%     writeDigitalPin(mypi,motor(2),1);
%     pause(delay);
%     i = i + 1;
%     current_position = initial_position + i * motor(5);
%     set(handles.edit_PositionY,'String',num2str(current_position));
    
    img = snapshot(mycam);
    imshow(img, 'Parent', handles.axes_live_cam);
    drawnow;
end
configurePin(mypi,motor(2),'DigitalOutput');


% --- Executes on button press in pushbutton_mY.
function pushbutton_mY_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_mY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global mypi
global dir_y
global motor_y
global mycam

motor = motor_y;
Dir = dir_y;
%motor = [enbl_pin step_pin dir_pin check_pin resolution sweep delay_min delay_max];

writeDigitalPin(mypi,motor(1),0);
writeDigitalPin(mypi,motor(3),Dir);

freq_menu = cellstr(get(handles.popupmenu_PWM,'String'));
freq = str2double(freq_menu{get(handles.popupmenu_PWM,'Value')});

%compute velocity
% velocity = get(handles.slider_velocity,'Value');
% set(handles.edit_velocity,'String', num2str(int8(velocity*100)));
% delay = (motor(7) - motor(8))/(1-0) * velocity + motor(8);

%get value
%initial_position = str2double(get(handles.edit_PositionY,'String'));

i=0;

configurePin(mypi, motor(2), 'PWM');
writePWMDutyCycle(mypi, motor(2), 0.5);
writePWMFrequency(mypi, motor(2), freq);

while get(hObject,'Value') == 1 && readDigitalPin(mypi,motor(4)) == 0
    %disp(rand)
    %pause(0.5)
%     if readDigitalPin(mypi,motor(4)) == 1
%         msgbox('Motor position out of range', 'Error','error');
%         break
%     end
%     writeDigitalPin(mypi,motor(2),0);
%     pause(delay);
%     writeDigitalPin(mypi,motor(2),1);
%     pause(delay);
%     i = i + 1;
%     current_position = initial_position - i * motor(5);
%     set(handles.edit_PositionY,'String',num2str(current_position));
    
    img = snapshot(mycam);
    imshow(img, 'Parent', handles.axes_live_cam);
    drawnow;
end
configurePin(mypi,motor(2),'DigitalOutput');


% --- Executes on button press in pushbutton_mX.
function pushbutton_mX_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_mX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% hObject    handle to pushbutton_pX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global mypi
global dir_x
global motor_x
global mycam

motor = motor_x;
Dir = dir_x;

%motor = [enbl_pin step_pin dir_pin check_pin resolution sweep delay_min delay_max];

writeDigitalPin(mypi,motor(1),0);
writeDigitalPin(mypi,motor(3),Dir);

%compute velocity

freq_menu = cellstr(get(handles.popupmenu_PWM,'String'));
freq = str2double(freq_menu{get(handles.popupmenu_PWM,'Value')});

%freq = get(handles.slider_velocity,'Value');

% set(handles.edit_velocity,'String', num2str(int8(velocity*100)));
% delay = (motor(7) - motor(8))/(1-0) * velocity + motor(8);

%get value
% initial_position = str2double(get(handles.edit_PositionY,'String'));

i=0;

configurePin(mypi, motor(2), 'PWM');
writePWMDutyCycle(mypi, motor(2), 0.5);
writePWMFrequency(mypi, motor(2), freq);

while get(hObject,'Value') == 1 && readDigitalPin(mypi,motor(4)) == 0
    %disp(rand)
    %pause(0.5)
%     if readDigitalPin(mypi,motor(4)) == 1
%         msgbox('Motor position out of range', 'Error','error');
%         break
%     end
%     writeDigitalPin(mypi,motor(2),0);
%     pause(delay);
%     writeDigitalPin(mypi,motor(2),1);
%     pause(delay);
%     i = i + 1;
%     current_position = initial_position - i * motor(5);
%     set(handles.edit_PositionX,'String',num2str(current_position));
%     
    img = snapshot(mycam);
    imshow(img, 'Parent', handles.axes_live_cam);
    drawnow;
end
configurePin(mypi,motor(2),'DigitalOutput');

% --- Executes on button press in pushbutton_pX.
function pushbutton_pX_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_pX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global mypi
global dir_x
global motor_x
global mycam

motor = motor_x;
Dir = not(dir_x);

%motor = [enbl_pin step_pin dir_pin check_pin resolution sweep delay_min delay_max];

writeDigitalPin(mypi,motor(1),0);
writeDigitalPin(mypi,motor(3),Dir);

%compute velocity
freq_menu = cellstr(get(handles.popupmenu_PWM,'String'));
freq = str2double(freq_menu{get(handles.popupmenu_PWM,'Value')});

% set(handles.edit_velocity,'String', num2str(int8(velocity*100)));
% delay = (motor(7) - motor(8))/(1-0) * velocity + motor(8);

%get value
%initial_position = str2double(get(handles.edit_PositionY,'String'));

i=0;

configurePin(mypi, motor(2), 'PWM');
writePWMDutyCycle(mypi, motor(2), 0.5);
writePWMFrequency(mypi, motor(2), freq);

while get(hObject,'Value') == 1 
    %disp(rand)
    %pause(0.5)
%     if readDigitalPin(mypi,motor(4)) == 1
%         msgbox('Motor position out of range', 'Error','error');
%         break
%     end
%     writeDigitalPin(mypi,motor(2),0);
%     pause(delay);
%     writeDigitalPin(mypi,motor(2),1);
%     pause(delay);
%     i = i + 1;
%     current_position = initial_position + i * motor(5);
%     set(handles.edit_PositionX,'String',num2str(current_position));
    
    img = snapshot(mycam);
    imshow(img, 'Parent', handles.axes_live_cam);
    drawnow;
end
configurePin(mypi,motor(2),'DigitalOutput');

% --- Executes on button press in pushbutton_pZ.
function pushbutton_pZ_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_pZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global mypi
global dir_z
global motor_z
global mycam

motor = motor_z;
Dir = not(dir_z);

%motor = [enbl_pin step_pin dir_pin check_pin resolution sweep delay_min delay_max];

writeDigitalPin(mypi,motor(1),0);
writeDigitalPin(mypi,motor(3),Dir);

%compute velocity
freq_menu = cellstr(get(handles.popupmenu_PWM,'String'));
freq = str2double(freq_menu{get(handles.popupmenu_PWM,'Value')});

% set(handles.edit_velocity,'String', num2str(int8(velocity*100)));
% delay = (motor(7) - motor(8))/(1-0) * velocity + motor(8);

%get value
%initial_position = str2double(get(handles.edit_PositionY,'String'));

i=0;

configurePin(mypi, motor(2), 'PWM');
writePWMDutyCycle(mypi, motor(2), 0.5);
writePWMFrequency(mypi, motor(2), freq);

while get(hObject,'Value') == 1 
    %disp(rand)
    %pause(0.5)
%     if readDigitalPin(mypi,motor(4)) == 1
%         msgbox('Motor position out of range', 'Error','error');
%         break
%     end
%     writeDigitalPin(mypi,motor(2),0);
%     pause(delay);
%     writeDigitalPin(mypi,motor(2),1);
%     pause(delay);
%     i = i + 1;
%     current_position = initial_position + i * motor(5);
%     set(handles.edit_PositionX,'String',num2str(current_position));
    
    img = snapshot(mycam);
    imshow(img, 'Parent', handles.axes_live_cam);
    drawnow;
end
configurePin(mypi,motor(2),'DigitalOutput');

% --- Executes on button press in pushbutton_mZ.
function pushbutton_mZ_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_mZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global mypi
global dir_z
global motor_z
global mycam

motor = motor_z;
Dir = dir_z;

%motor = [enbl_pin step_pin dir_pin check_pin resolution sweep delay_min delay_max];

writeDigitalPin(mypi,motor(1),0);
writeDigitalPin(mypi,motor(3),Dir);

%compute velocity
freq_menu = cellstr(get(handles.popupmenu_PWM,'String'));
freq = str2double(freq_menu{get(handles.popupmenu_PWM,'Value')});

% set(handles.edit_velocity,'String', num2str(int8(velocity*100)));
% delay = (motor(7) - motor(8))/(1-0) * velocity + motor(8);

%get value
%initial_position = str2double(get(handles.edit_PositionY,'String'));

i=0;

configurePin(mypi, motor(2), 'PWM');
writePWMDutyCycle(mypi, motor(2), 0.5);
writePWMFrequency(mypi, motor(2), freq);

while get(hObject,'Value') == 1 && readDigitalPin(mypi,motor(4)) == 0
    %disp(rand)
    %pause(0.5)
%     if readDigitalPin(mypi,motor(4)) == 1
%         msgbox('Motor position out of range', 'Error','error');
%         break
%     end
%     writeDigitalPin(mypi,motor(2),0);
%     pause(delay);
%     writeDigitalPin(mypi,motor(2),1);
%     pause(delay);
%     i = i + 1;
%     current_position = initial_position + i * motor(5);
%     set(handles.edit_PositionX,'String',num2str(current_position));
    
    img = snapshot(mycam);
    imshow(img, 'Parent', handles.axes_live_cam);
    drawnow;
end
configurePin(mypi,motor(2),'DigitalOutput');


function edit_PositionX_Callback(hObject, eventdata, handles)
% hObject    handle to edit_PositionX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_PositionX as text
%        str2double(get(hObject,'String')) returns contents of edit_PositionX as a double


% --- Executes during object creation, after setting all properties.
function edit_PositionX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_PositionX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_PositionY_Callback(hObject, eventdata, handles)
% hObject    handle to edit_PositionY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_PositionY as text
%        str2double(get(hObject,'String')) returns contents of edit_PositionY as a double


% --- Executes during object creation, after setting all properties.
function edit_PositionY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_PositionY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_PositionZ_Callback(hObject, eventdata, handles)
% hObject    handle to edit_PositionZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_PositionZ as text
%        str2double(get(hObject,'String')) returns contents of edit_PositionZ as a double


% --- Executes during object creation, after setting all properties.
function edit_PositionZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_PositionZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_PositionTheta_Callback(hObject, eventdata, handles)
% hObject    handle to edit_PositionTheta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_PositionTheta as text
%        str2double(get(hObject,'String')) returns contents of edit_PositionTheta as a double


% --- Executes during object creation, after setting all properties.
function edit_PositionTheta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_PositionTheta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_PositionDepth_Callback(hObject, eventdata, handles)
% hObject    handle to edit_PositionDepth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_PositionDepth as text
%        str2double(get(hObject,'String')) returns contents of edit_PositionDepth as a double


% --- Executes during object creation, after setting all properties.
function edit_PositionDepth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_PositionDepth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_pTheta.
function pushbutton_pTheta_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_pTheta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton_mTheta.
function pushbutton_mTheta_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_mTheta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.
function slider_velocity_Callback(hObject, eventdata, handles)
velocity = get(handles.slider_velocity,'Value');
set(handles.edit_velocity,'String', num2str(int8(velocity*100)));


% --- Executes during object creation, after setting all properties.
function slider_velocity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_velocity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit_velocity_Callback(hObject, eventdata, handles)
% hObject    handle to edit_velocity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_velocity as text
%        str2double(get(hObject,'String')) returns contents of edit_velocity as a double


% --- Executes during object creation, after setting all properties.
function edit_velocity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_velocity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_Save_Pos.
function pushbutton_Save_Pos_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Save_Pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% axes(handles.axes_sensor_tile);
% [x,y] = ginput(1)


% --- Executes on button press in pushbutton_Reset.
function pushbutton_Reset_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton_Start.
function pushbutton_Start_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global motor_theta mypi dir_z motor_z mycam flagz motor_x motor_y dir_x dir_y motor_depth dir_depth Rmd_step;

%if flagz == true
    set_angle=str2num(get(handles.edit_angle,'String'));
    
    
    e_theta = 2;
    Res = motor_theta(5);
    theta_f = set_angle;
    pause(0.5);
    configurePin(mypi,motor_theta(1),'DigitalOutput');
    %%% read value and plot acelerometer and angle %%%
    handles.dat1.update('agm');
    theta_current = atand(handles.dat1.acc(2)/handles.dat1.acc(3));
    %motor_theta(1)
    pause(0.5);
    writeDigitalPin(mypi,motor_theta(1),0);
    
    delay_min=0.001;
    while abs(theta_current - theta_f) > e_theta
        theta_i = theta_current;
        delta = theta_current - theta_f;
        Nstep = int16(delta / Res);
        %[end_move, N] = move_motor(mypi, motor_theta, abs(NStep), gt(Nstep,0), vel);
        %%% function move %%%
        writeDigitalPin(mypi,motor_theta(3),gt(Nstep,0));
        pause(1);
        %         for i = 1:abs(Nstep)
        %             writeDigitalPin(mypi,motor_theta(2),0);
        %             pause(delay_min);
        %             writeDigitalPin(mypi,motor_theta(2),1);
        %             pause(delay_min);
        %         end
        
        %p=gcp();
        %ftheta = parfeval(p,@init_theta,1,mypi,motor_theta,motor_theta(7), abs(Nstep));
        i=0;
        delay_min = 0.001;
        for i = 1:abs(Nstep)
            pause(delay_min);
            writeDigitalPin(mypi,motor_theta(2),0);
            pause(delay_min);
            writeDigitalPin(mypi,motor_theta(2),1);
            
            %    while isempty(fetchNext(ftheta,0))
            %handles.dat1.update('agm');
            %axis(handles.axes_sensor_tile,[0 Inf -Inf Inf]);
            %addpoints(handles.h1,double(i),double(atand(handles.dat1.acc(2)/handles.dat1.acc(3))));
            %drawnow;
            %i=i+1;
        end
        %    end
        %writeDigitalPin(mypi,motor_theta(1),1);
        %%% end function %%%
        
        pause(2);
        
        %%% read value %%%
        
        handles.dat1.update('agm');
        handles.dat1.update('agm');
        handles.dat1.update('agm');
        handles.dat1.update('agm');
        handles.dat1.update('agm');

        theta_current = atand(handles.dat1.acc(2)/handles.dat1.acc(3));
        
        while theta_current == theta_i
            handles.dat1.update('agm');
            theta_current = atand(handles.dat1.acc(2)/handles.dat1.acc(3));
        end
        
        %theta_current = atand(handles.dat1.acc(2)/handles.dat1.acc(3));
        theta_current-theta_f
        
        %Res = abs(theta_current - theta_i)/double(i);
    end
    
    %p=gcp();
    %ftemp=parfeval(p,@config_motor_PWM,1,mypi, 14, 25, 12, 13, 0.5/513, 10, 0, not(dir_z), 0.001, 0.1, 200);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    clearpoints(handles.h1);
    clearpoints(handles.h2);
    clearpoints(handles.h3);
    
    teste1=zeros(1,10000);
    teste2=zeros(1,10000);
    teste3=zeros(1,10000);
    
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    
    writeDigitalPin(mypi,motor_z(1),0);
    writeDigitalPin(mypi,motor_z(3),not(dir_z));
    
    configurePin(mypi, motor_z(2), 'PWM');
    writePWMDutyCycle(mypi, motor_z(2), 0.5);
    writePWMFrequency(mypi, motor_z(2), 100);
    
    
    i=1;
    sigma = 5;
    while 1
        handles.dat1.update('agm');
        if handles.dat1.sound > 90 && handles.dat1.sound < 120 && get(handles.checkbox_Alarm,'Value') == 1
            configurePin(mypi, motor_z(2), 'DigitalOutput');
            mbox = msgbox('Too noisy!','ALARM!');
            pause(1);
            delete(mbox);
            return;
        end
        if norm(handles.dat1.gyr) > 100 && get(handles.Gyro_alarm,'Value') == 1
            configurePin(mypi, motor_z(2), 'DigitalOutput');
            mbox = msgbox('Too shaky!','ALARM!');
            pause(1);
            delete(mbox);
            return;
        end
        axis(handles.axes_sensor_tile,[0 Inf -Inf Inf]);
        %addpoints(handles.h1,double(i),handles.dat1.gyr(1));
        %addpoints(handles.h2,double(i),handles.dat1.gyr(2));
        %addpoints(handles.h3,double(i),norm(handles.dat1.sound));
        addpoints(handles.h3,double(i),handles.dat1.mag(3));

        %img = snapshot(mycam);
        %imshow(img, 'Parent', handles.axes_live_cam);
        drawnow;
        %teste1(i)=handles.dat1.gyr(1);
        %teste2(i)=handles.dat1.gyr(2);
        teste3(i)=handles.dat1.mag(3);
        if i > 150
            baseline=mean(teste3(i-149:i-50));
            sd = std(teste3(i-149:i-50));
            addpoints(handles.h1,double(i),baseline+sigma*sd);
            addpoints(handles.h2,double(i),baseline-sigma*sd);
%             img = snapshot(mycam);
%             imshow(img, 'Parent', handles.axes_live_cam);
            drawnow;
        end
        if  i > 150 && double(abs(baseline - handles.dat1.mag(3))) > sigma*double(sd)
%             img = snapshot(mycam);
%             imshow(img, 'Parent', handles.axes_live_cam);
%             drawnow;
            break
        end
        i=i+1;
    end
    
    configurePin(mypi, motor_z(2), 'DigitalOutput');
    writeDigitalPin(mypi,motor_z(3),dir_z);
    
    Nstepz = 1026;
    
    for i=1:int16(1*Nstepz+Nstepz*str2double(get(handles.tolerance_edit,'String')))
        writeDigitalPin(mypi,motor_z(2),0);
        pause(delay_min);
        writeDigitalPin(mypi,motor_z(2),1);
        pause(delay_min);
    end
    
    status_box = msgbox('Sample Contact Done ...', 'Sample Contact');
    pause(1);
    delete(status_box);
    
    %flagz=true;

%     img = snapshot(mycam);
%     imshow(img, 'Parent', handles.axes_select_point);
%     drawnow;


% 
% mbox = msgbox('Select first calibration point','1st point');
% pause(1);
% delete(mbox);
% 
% axes(handles.axes_select_point);
% [pt1_x,pt1_y] = ginput(1);
% 
% img = snapshot(mycam);
% img = insertMarker(img, [pt1_x, pt1_y],'o');
% imshow(img, 'Parent', handles.axes_select_point);
% drawnow;
% 
% configurePin(mypi, motor_x(2), 'DigitalOutput');
% writeDigitalPin(mypi,motor_x(3),not(dir_x));
% 
% for i=1:Nstepz
%     writeDigitalPin(mypi,motor_x(2),0);
%     pause(delay_min);
%     writeDigitalPin(mypi,motor_x(2),1);
%     pause(delay_min);
% end
% 
% configurePin(mypi, motor_y(2), 'DigitalOutput');
% writeDigitalPin(mypi,motor_y(3),not(dir_y));
% 
% for i=1:Nstepz
%     writeDigitalPin(mypi,motor_y(2),0);
%     pause(delay_min);
%     writeDigitalPin(mypi,motor_y(2),1);
%     pause(delay_min);
% end
% 
% img = snapshot(mycam);
% img = insertMarker(img, [pt1_x, pt1_y],'o');
% imshow(img, 'Parent', handles.axes_select_point);
% drawnow;
% 
% mbox = msgbox('Select second calibration point','2nd point');
% pause(1);
% delete(mbox);
% 
% [pt2_x,pt2_y] = ginput(1);
% 
% img = snapshot(mycam);
% img = insertMarker(img, [pt1_x, pt1_y],'o');
% img = insertMarker(img, [pt2_x, pt2_y],'o');
% imshow(img, 'Parent', handles.axes_select_point);
% drawnow;
% 
% Nstepx = Nstepz/abs(pt2_x-pt1_x);
% Nstepy = Nstepz/abs(pt2_x-pt1_x);
% 
% img = snapshot(mycam);
% img = insertMarker(img, [pt1_x, pt1_y],'o');
% img = insertMarker(img, [pt2_x, pt2_y],'o');
% imshow(img, 'Parent', handles.axes_select_point);
% drawnow;
% 
% mbox=msgbox('Select target point','Target point');
% pause(1);
% delete(mbox);
% 
% [ptar_x,ptar_y] = ginput(1);
% 
% moveX = Nstepx*(ptar_x-pt2_x);
% moveY = Nstepy*(ptar_y-pt2_y);
% 
% writeDigitalPin(mypi,motor_x(3),gt(moveX,0));
% writeDigitalPin(mypi,motor_y(3),gt(moveY,0));
% 
% for i=1:int16(abs(moveX))
%     writeDigitalPin(mypi,motor_x(2),0);
%     pause(delay_min);
%     writeDigitalPin(mypi,motor_x(2),1);
%     pause(delay_min);
% end
% 
% for i=1:int16(abs(moveY))
%     writeDigitalPin(mypi,motor_y(2),0);
%     pause(delay_min);
%     writeDigitalPin(mypi,motor_y(2),1);
%     pause(delay_min);
% end
% 
% choice = questdlg('Is this the correct point?', 'Confirm point', 'Yes', 'No');
% 
% switch choice 
%     case 'Yes'
%     case 'No'
        mbox = msgbox('Please select the point to insert','Manual mode engaged');
        pause(1);
        delete(mbox);
        while get(handles.pushbutton_Save_Pos,'Value')==0
            img = snapshot(mycam);
            imshow(img, 'Parent', handles.axes_live_cam);
            drawnow;
        end
% end
% 
set_depth = str2double(get(handles.edit_depth,'String'));

% %move X Back for insertion
% moveX = int16(0.001*set_depth*cosd(set_angle)*1026);
% 
% writeDigitalPin(mypi,motor_x(3),dir_x);
% 
% for i=1:int16(abs(moveX))
%     writeDigitalPin(mypi,motor_x(2),0);
%     pause(delay_min);
%     writeDigitalPin(mypi,motor_x(2),1);
%     pause(delay_min);
%     img = snapshot(mycam);
%     imshow(img, 'Parent', handles.axes_live_cam);
%     drawnow;
% end

pause(1);

status_box = msgbox('Starting Insertion (Z axis) ...', 'Sample Insertion');

configurePin(mypi, motor_z(2), 'DigitalOutput');
writeDigitalPin(mypi,motor_z(3),not(dir_z));

for i=1:int16(Nstepz*str2double(get(handles.tolerance_edit,'String')))
    handles.dat1.update('agm');
    if handles.dat1.sound > 90 && handles.dat1.sound < 120 && get(handles.checkbox_Alarm,'Value') == 1
        configurePin(mypi, motor_z(2), 'DigitalOutput');
        mbox = msgbox('Too noisy!','ALARM!');
        pause(1);
        delete(mbox);
        return;
    end
    if norm(handles.dat1.gyr) > 100  && get(handles.Gyro_alarm,'Value') == 1
        configurePin(mypi, motor_z(2), 'DigitalOutput');
        mbox = msgbox('Too shaky!','ALARM!');
        pause(1);
        delete(mbox);
        return;
    end
    writeDigitalPin(mypi,motor_z(2),0);
    pause(delay_min*0.1);
    writeDigitalPin(mypi,motor_z(2),1);
    pause(delay_min*0.1);
    img = snapshot(mycam);
    imshow(img, 'Parent', handles.axes_live_cam);
    drawnow;
end

configurePin(mypi, 15, 'DigitalOutput');
configurePin(mypi, 18, 'DigitalOutput');
writeDigitalPin(mypi,18,not(dir_depth));

dsteps=int16(str2double(get(handles.edit_depth,'String'))*0.001*Rmd_step);

pause(1);
delete(status_box);
status_box = msgbox('Starting Insertion (Depth Axis) ...', 'Sample Insertion');

for i=1:dsteps
    handles.dat1.update('agm');
    if (handles.dat1.sound > 90 && handles.dat1.sound < 120) && get(handles.checkbox_Alarm,'Value') == 1
        configurePin(mypi, 15, 'DigitalOutput');
        mbox = msgbox('Too noisy!','ALARM!');
        pause(1);
        delete(mbox);
        return;
    end
    if norm(handles.dat1.gyr) > 100 && get(handles.Gyro_alarm,'Value') == 1
        configurePin(mypi, 15, 'DigitalOutput');
        mbox = msgbox('Too shaky!','ALARM!');
        pause(1);
        delete(mbox);
        return;
    end
    writeDigitalPin(mypi,15,0);
    pause(delay_min*0.1);
    writeDigitalPin(mypi,15,1);
    pause(delay_min*0.1);
    img = snapshot(mycam);
    imshow(img, 'Parent', handles.axes_live_cam);
    drawnow;
end

pause(1);
delete(status_box);
status_box = msgbox('Insertion Done...', 'Sample Insertion');
pause(2);
delete(status_box);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



function edit_angle_Callback(hObject, eventdata, handles)
% hObject    handle to edit_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_angle as text
%        str2double(get(hObject,'String')) returns contents of edit_angle as a double


% --- Executes during object creation, after setting all properties.
function edit_angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_angle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_depth_Callback(hObject, eventdata, handles)
% hObject    handle to edit_depth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_depth as text
%        str2double(get(hObject,'String')) returns contents of edit_depth as a double


% --- Executes during object creation, after setting all properties.
function edit_depth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_depth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_select_point.
function pushbutton_select_point_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_select_point (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit_point_y_Callback(hObject, eventdata, handles)
% hObject    handle to edit_point_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_point_y as text
%        str2double(get(hObject,'String')) returns contents of edit_point_y as a double


% --- Executes during object creation, after setting all properties.
function edit_point_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_point_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_point_x_Callback(hObject, eventdata, handles)
% hObject    handle to edit_point_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_point_x as text
%        str2double(get(hObject,'String')) returns contents of edit_point_x as a double


% --- Executes during object creation, after setting all properties.
function edit_point_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_point_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox_Alarm.
function checkbox_Alarm_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_Alarm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_Alarm



function edit_PositionX_save_Callback(hObject, eventdata, handles)
% hObject    handle to edit_PositionX_save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_PositionX_save as text
%        str2double(get(hObject,'String')) returns contents of edit_PositionX_save as a double


% --- Executes during object creation, after setting all properties.
function edit_PositionX_save_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_PositionX_save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_PositionY_Save_Callback(hObject, eventdata, handles)
% hObject    handle to edit_PositionY_Save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_PositionY_Save as text
%        str2double(get(hObject,'String')) returns contents of edit_PositionY_Save as a double


% --- Executes during object creation, after setting all properties.
function edit_PositionY_Save_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_PositionY_Save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_PositionZ_Save_Callback(hObject, eventdata, handles)
% hObject    handle to edit_PositionZ_Save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_PositionZ_Save as text
%        str2double(get(hObject,'String')) returns contents of edit_PositionZ_Save as a double


% --- Executes during object creation, after setting all properties.
function edit_PositionZ_Save_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_PositionZ_Save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_PositionTheta_Save_Callback(hObject, eventdata, handles)
% hObject    handle to edit_PositionTheta_Save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_PositionTheta_Save as text
%        str2double(get(hObject,'String')) returns contents of edit_PositionTheta_Save as a double


% --- Executes during object creation, after setting all properties.
function edit_PositionTheta_Save_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_PositionTheta_Save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_PositionDepth_Save_Callback(hObject, eventdata, handles)
% hObject    handle to edit_PositionDepth_Save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_PositionDepth_Save as text
%        str2double(get(hObject,'String')) returns contents of edit_PositionDepth_Save as a double


% --- Executes during object creation, after setting all properties.
function edit_PositionDepth_Save_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_PositionDepth_Save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tolerance_edit_Callback(hObject, eventdata, handles)
% hObject    handle to tolerance_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tolerance_edit as text
%        str2double(get(hObject,'String')) returns contents of tolerance_edit as a double


% --- Executes during object creation, after setting all properties.
function tolerance_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tolerance_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu_PWM.
function popupmenu_PWM_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_PWM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_PWM contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_PWM


% --- Executes during object creation, after setting all properties.
function popupmenu_PWM_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_PWM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Gyro_alarm.
function Gyro_alarm_Callback(hObject, eventdata, handles)
% hObject    handle to Gyro_alarm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Gyro_alarm



function show_temp_Callback(hObject, eventdata, handles)
% hObject    handle to show_temp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of show_temp as text
%        str2double(get(hObject,'String')) returns contents of show_temp as a double


% --- Executes during object creation, after setting all properties.
function show_temp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to show_temp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function show_hum_Callback(hObject, eventdata, handles)
% hObject    handle to show_hum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of show_hum as text
%        str2double(get(hObject,'String')) returns contents of show_hum as a double


% --- Executes during object creation, after setting all properties.
function show_hum_CreateFcn(hObject, eventdata, handles)
% hObject    handle to show_hum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function show_pres_Callback(hObject, eventdata, handles)
% hObject    handle to show_pres (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of show_pres as text
%        str2double(get(hObject,'String')) returns contents of show_pres as a double


% --- Executes during object creation, after setting all properties.
function show_pres_CreateFcn(hObject, eventdata, handles)
% hObject    handle to show_pres (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function show_edt_Callback(hObject, eventdata, handles)
% hObject    handle to show_edt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of show_edt as text
%        str2double(get(hObject,'String')) returns contents of show_edt as a double


% --- Executes during object creation, after setting all properties.
function show_edt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to show_edt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
