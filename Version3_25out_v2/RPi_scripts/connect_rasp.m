function [mypi,piON] = connect_rasp(ip, username, password, delay)

iconON = imread('checkmark.png');
status_box = msgbox('Connecting Raspberry Pi ...', 'Connecting');
piON = 1;

try
    mypi = raspi(ip,username,password);
catch
    delete(status_box)
    status_box = msgbox('Raspberry Connection Failed', 'Error','error');
    piON = 0;
    mypi = 0;
end

if piON == 1
    delete(status_box)
    status_box = msgbox('Raspberry Connected','Success','custom',iconON);
end

pause(delay)
delete(status_box)
end

