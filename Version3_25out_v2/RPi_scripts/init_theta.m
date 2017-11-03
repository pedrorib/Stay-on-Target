function [ status ] = init_theta( mypi,motor_theta,delay_min, Nstep )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
pause(1);
for i = 1:abs(Nstep)
    writeDigitalPin(mypi,motor_theta(2),0);
    pause(delay_min);
    writeDigitalPin(mypi,motor_theta(2),1);
    pause(delay_min);
end
pause(1);
status=1;
end

