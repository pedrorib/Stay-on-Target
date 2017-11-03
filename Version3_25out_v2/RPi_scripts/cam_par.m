function [ status ] = cam_par( mycam, handles )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
while 1
            img = snapshot(mycam);
            imshow(img, 'Parent', handles.axes_live_cam);
            drawnow;
end
status=1;
end

