function show_cam(handles, mycam, live)
if live == 1
    img = snapshot(mycam);
    imshow(img, 'Parent', handles.axes_live_cam);
    drawnow;
end
end