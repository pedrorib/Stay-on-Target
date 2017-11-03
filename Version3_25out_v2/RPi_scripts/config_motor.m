function motor = config_motor(mypi,enbl_pin, step_pin, dir_pin, check_pin, resolution, sweep, rel_pos, dir, delay_min, delay_max)

%resolution m / step

motor = [enbl_pin step_pin dir_pin check_pin resolution sweep delay_min delay_max];

for i = 1:3
    configurePin(mypi,motor(i),'DigitalOutput');
    writeDigitalPin(mypi,motor(i),0);
end

configurePin(mypi,motor(i+1),'DigitalInput');

if dir == 1 || dir == 0
    writeDigitalPin(mypi,motor(1),0);
    writeDigitalPin(mypi,motor(3),dir);
    j=0;
    while readDigitalPin(mypi,motor(4)) == 0
        writeDigitalPin(mypi,motor(2),0);
        pause(delay_min);
        writeDigitalPin(mypi,motor(2),1);
        pause(delay_min);
        %j=j+1
    end
    
    writeDigitalPin(mypi,motor(3),not(dir));
    NStep = int16(motor(6) * rel_pos / motor(5));
    
    for i = 1:NStep
        writeDigitalPin(mypi,motor(2),0);
        pause(delay_min);
        writeDigitalPin(mypi,motor(2),1);
        pause(delay_min);
        %i
    end
   
   
end

writeDigitalPin(mypi,motor(1),1);

end

