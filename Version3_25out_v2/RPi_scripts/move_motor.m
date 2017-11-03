function [end_move, N] = move_motor(mypi, motor, NStep, Dir, delay)
%motor = [enbl_pin step_pin dir_pin check_pin resolution sweep];
%resolution m/step

writeDigitalPin(mypi,motor(1),0);
writeDigitalPin(mypi,motor(3),Dir);
for i = 1:NStep
    if readDigitalPin(mypi,motor(4)) == 1
        end_move = 0;
        msgbox('Motor position out of range', 'Error','error');
        break
    else
        end_move = 1;
    end
    writeDigitalPin(mypi,motor(2),0);
    pause(delay);
    writeDigitalPin(mypi,motor(2),1);
    pause(delay);
end

N = i;
writeDigitalPin(mypi,motor(1),1);

end

