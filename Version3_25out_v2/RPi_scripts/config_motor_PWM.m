function motor = config_motor_PWM(mypi,enbl_pin, step_pin, dir_pin, check_pin, resolution, sweep, rel_pos, dir, delay_min, delay_max, freq)

%resolution m / step

motor = [enbl_pin step_pin dir_pin check_pin resolution sweep delay_min delay_max];

for i = 1:3
    configurePin(mypi,motor(i),'DigitalOutput');
    writeDigitalPin(mypi,motor(i),0);
end

configurePin(mypi,motor(i+1),'DigitalInput');

%set enable and direction
writeDigitalPin(mypi,motor(1),0);
writeDigitalPin(mypi,motor(3),dir);

% move motor with PWM until hits the button
configurePin(mypi, motor(2), 'PWM');
writePWMDutyCycle(mypi, motor(2), 0.5);
writePWMFrequency(mypi, motor(2), freq);

%stop moving when button is high
while readDigitalPin(mypi,motor(4)) == 0
end

%Stop PWM
configurePin(mypi,motor(2),'DigitalOutput');
writeDigitalPin(mypi,motor(2),0);

% move to initial position
writeDigitalPin(mypi,motor(3),not(dir));
NStep = int16(motor(6) * rel_pos / motor(5));

for i = 1:NStep
    writeDigitalPin(mypi,motor(2),0);
    pause(delay_min);
    writeDigitalPin(mypi,motor(2),1);
    pause(delay_min);
end

writeDigitalPin(mypi,motor(1),1);

