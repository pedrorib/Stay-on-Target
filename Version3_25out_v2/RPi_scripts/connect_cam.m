function [mycam, camON] = connect_cam(mypi, resolution, quality, rotation, rate, brightness, contrast, saturation, sharpness, delay)

iconON = imread('checkmark.png');
status_box = msgbox('Connecting Camera ...', 'Connecting');
camON = 1;

try
    mycam = cameraboard(mypi,'Resolution',resolution,...
                             'Quality',quality,...
                             'Rotation',rotation,...
                             'FrameRate',rate,...
                             'Brightness',brightness,...
                             'Contrast',contrast,...
                             'Saturation',saturation,...
                             'Sharpness',sharpness);
catch
    delete(status_box)
    status_box = msgbox('Camera Connection Failed', 'Error','error');
    camON = 0;
    mycam = 0;
end

if camON == 1
    delete(status_box)
    status_box = msgbox('Camera Connected','Success','custom',iconON);
end

pause(delay)
delete(status_box)
end

