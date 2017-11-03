function [ i, change_sensor ] = show_sensor( change_sensor, handles, restart_ST,i )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
if restart_ST ~= 0
    sensor2 = get(handles.popupmenu_ST,'Value');
    %sensor2=char(sensor2);
    
    if change_sensor == true
        clearpoints(handles.h1);
        clearpoints(handles.h2);
        clearpoints(handles.h3);
        i=0;
        change_sensor = false;
    end
    switch sensor2
        case 1
            handles.dat1.update('agm');
            if i<200
                axis(handles.axes_sensor_tile,[0 200 -Inf Inf]);
                addpoints(handles.h1,i,handles.dat1.acc(1))
                addpoints(handles.h2,i,handles.dat1.acc(2))
                addpoints(handles.h3,i,handles.dat1.acc(3))
            else
                axis(handles.axes_sensor_tile,[i-200 i -Inf Inf])
                addpoints(handles.h1,i,handles.dat1.acc(1))
                addpoints(handles.h2,i,handles.dat1.acc(2))
                addpoints(handles.h3,i,handles.dat1.acc(3))
            end
        case 2
            handles.dat1.update('agm');
            if i<200
                axis(handles.axes_sensor_tile,[0 200 -Inf Inf]);
                addpoints(handles.h1,i,handles.dat1.gyr(1))
                addpoints(handles.h2,i,handles.dat1.gyr(2))
                addpoints(handles.h3,i,handles.dat1.gyr(3))
            else
                axis(handles.axes_sensor_tile,[i-200 i -Inf Inf])
                addpoints(handles.h1,i,handles.dat1.gyr(1))
                addpoints(handles.h2,i,handles.dat1.gyr(2))
                addpoints(handles.h3,i,handles.dat1.gyr(3))
            end
        case 3
            handles.dat1.update('agm');
            if i<200
                axis(handles.axes_sensor_tile,[0 200 -Inf Inf]);
                addpoints(handles.h1,i,handles.dat1.mag(1))
                addpoints(handles.h2,i,handles.dat1.mag(2))
                addpoints(handles.h3,i,handles.dat1.mag(3))
            else
                axis(handles.axes_sensor_tile,[i-200 i -Inf Inf])
                addpoints(handles.h1,i,handles.dat1.mag(1))
                addpoints(handles.h2,i,handles.dat1.mag(2))
                addpoints(handles.h3,i,handles.dat1.mag(3))
            end
        case 5
            handles.dat1.update('pt');
            if i<200
                axis(handles.axes_sensor_tile,[0 200 -Inf Inf]);
                addpoints(handles.h1,i,handles.dat1.temp2);
            else
                axis(handles.axes_sensor_tile,[i-200 i -Inf Inf])
                addpoints(handles.h1,i,handles.dat1.temp2);
            end
        case 4
            handles.dat1.update('pt');
            if i<200
                axis(handles.axes_sensor_tile,[0 200 -Inf Inf]);
                addpoints(handles.h1,i,handles.dat1.hum);
            else
                axis(handles.axes_sensor_tile,[i-200 i -Inf Inf])
                addpoints(handles.h1,i,handles.dat1.hum);
            end
        case 6
            handles.dat1.update('agm');
            if i<200
                axis(handles.axes_sensor_tile,[0 200 -Inf Inf]);
                if handles.dat1.sound > 10 || handles.dat1.sound < 250
                    addpoints(handles.h1,i,handles.dat1.sound);
                end
            else
                axis(handles.axes_sensor_tile,[i-200 i -Inf Inf])
                if handles.dat1.sound > 10 || handles.dat1.sound < 250
                    addpoints(handles.h1,i,handles.dat1.sound);
                end
            end
        case 7
            handles.dat1.update('agm');
            if i<200
                axis(handles.axes_sensor_tile,[0 200 -Inf Inf]);
                addpoints(handles.h1,i,handles.dat1.trash);
            else
                axis(handles.axes_sensor_tile,[i-200 i -Inf Inf])
                addpoints(handles.h1,i,handles.dat1.trash);
            end
    end
    drawnow;
    i=i+1;
end
end
