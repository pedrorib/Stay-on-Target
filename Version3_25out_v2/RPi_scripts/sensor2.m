classdef sensor2 < handle
    %pt - Fetches pressure and temperature from ST sensor and returns these
    %as elements of this class as scalar signed integers
    
    properties
        tsagm %Inertial sensor timestamp
        tspt %Pressure/Temperature timestamp
        tssnd %Sound timestamp
        acc %ST sensor ouput for acceleration
        gyr %ST sensor output for angular velocity
        mag %ST sensor output for magnetometer
        temp1 %ST sensor output for temperature
        pres %ST sensor output for pressure
        sound %ST sensor output for sound level
        trash %Unused data
        temp2;
        hum;
    end
    properties(Access = private)
        RPi
        BT;
    end
    
    methods
        function out = sensor2(pihandle,BTAddress)
            %Initializes connection to the ST sensor through the raspberry pi.
            %pt(pihandle,BTAddress):
            %pihandle - handle for raspberry pi, returned by raspi function
            %BTAddress - ST sensor MAC Address
            out.RPi=pihandle;
            out.BT=BTAddress;
            out.acc=0;
            out.gyr=0;
            out.mag=0;
            out.temp1=0;
            out.pres=0;
            out.sound=0;
            out.tsagm=0;
            out.tspt=0;
            out.tssnd=0;
            out.hum=0;
            out.temp2=0;
            test=[1];
            system(pihandle,['. ~/pasta_bt/exp_init.sh ' BTAddress ' > /dev/null &']);
            pause(3);
            while(length(test)==1)
                test=sscanf(system(pihandle, '. ~/pasta_bt/conv_dados_v2.sh 0 0 0'),'%x');
            end
            system(pihandle, '. ~/pasta_bt/conv_dados_v2.sh 0 0 0');
        end
    end
    methods
        function self = update(self,meas1,meas2)
            complete = 0;
            if nargin == 2 && strcmp(meas1, 'agm') == 1
                out=sscanf(system(self.RPi, '. ~/pasta_bt/conv_dados_v2.sh 0 1 0'),'%x');
                if length(out) < 5
                    return;
                end
                if length(out) == 31
                    complete=1;
                else
                for i=2:2:20
                    aux1=out(i-1);
                    aux2=out(i);
                    r_aux=aux1+aux2*256;
                    if i<21
                        if r_aux > 32768
                            r_aux=r_aux-65536;
                        end
                    end
                    switch i
                        case 2
                            self.tsagm=r_aux;
                        case 4
                            self.acc(1)=r_aux;
                        case 6
                            self.acc(2)=r_aux;
                        case 8
                            self.acc(3)=r_aux;
                        case 10
                            self.gyr(1)=r_aux;
                        case 12
                            self.gyr(2)=r_aux;
                        case 14
                            self.gyr(3)=r_aux;
                        case 16
                            self.mag(1)=r_aux;
                        case 18
                            self.mag(2)=r_aux;
                        case 20
                            self.mag(3)=r_aux;
                    end
                end
                self.tssnd=out(21)+out(22)*256;
                self.sound=out(23);
                end
            end
            if nargin == 2 && strcmp(meas1, 'pt') == 1
                out=sscanf(system(self.RPi, '. ~/pasta_bt/conv_dados_v2.sh 1 0 0'),'%x');
                if length(out) < 5
                    return;
                end
                if length(out) == 31
                    complete=1;
                else                
                for i=2:2:12
                    aux1=out(i-1);
                    aux2=out(i);
                    r_aux=aux1+aux2*256;
                    if i<21
                        if r_aux > 32768
                            r_aux=r_aux-65536;
                        end
                    end
                    switch i
                        case 2
                            self.tspt=r_aux;
                        case 4
                            self.pres=r_aux;
                        case 6
                            self.trash=r_aux;
                        case 8
                            self.hum=r_aux;
                        case 10
                            self.temp2=r_aux;
                        case 12
                            self.temp1=r_aux;
                    end
                end
                self.tssnd=out(9)+out(10)*256;
                self.sound=out(11);
                end
            end
            if complete == 1 && nargin == 2
                for i=2:2:34
                    aux1=out(i-1);
                    aux2=out(i);
                    r_aux=aux1+aux2*256;
                    if i<21
                        if r_aux > 32768
                            r_aux=r_aux-65536;
                        end
                    end
                    switch i
                        case 2
                            self.tsagm=r_aux;
                        case 4
                            self.acc(1)=r_aux;
                        case 6
                            self.acc(2)=r_aux;
                        case 8
                            self.acc(3)=r_aux;
                        case 10
                            self.gyr(1)=r_aux;
                        case 12
                            self.gyr(2)=r_aux;
                        case 14
                            self.gyr(3)=r_aux;
                        case 16
                            self.mag(1)=r_aux;
                        case 18
                            self.mag(2)=r_aux;
                        case 20
                            self.mag(3)=r_aux;
                        case 22
                            self.tspt=r_aux;
                        case 24
                            self.pres=r_aux;
                        case 26
                            self.trash=r_aux;
                        case 28
                            self.hum=r_aux;
                        case 30
                            self.temp2=r_aux;
                        case 32
                            self.temp1=r_aux;
                    end
                end
                self.tssnd=out(33)+out(34)*256;
                self.sound=out(35);
            end
            if nargin == 1 || (nargin == 3 && ((strcmp(meas1,'agm') == 1 && strcmp(meas2,'pt') == 1) || (strcmp(meas2,'agm') == 1 && strcmp(meas1,'pt') == 1)))
                out=sscanf(system(self.RPi, '. ~/pasta_bt/conv_dados_v2.sh 0 0 0'),'%x');
                for i=2:2:34
                    aux1=out(i-1);
                    aux2=out(i);
                    r_aux=aux1+aux2*256;
                    if i<21
                        if r_aux > 32768
                            r_aux=r_aux-65536;
                        end
                    end
                    switch i
                        case 2
                            self.tsagm=r_aux;
                        case 4
                            self.acc(1)=r_aux;
                        case 6
                            self.acc(2)=r_aux;
                        case 8
                            self.acc(3)=r_aux;
                        case 10
                            self.gyr(1)=r_aux;
                        case 12
                            self.gyr(2)=r_aux;
                        case 14
                            self.gyr(3)=r_aux;
                        case 16
                            self.mag(1)=r_aux;
                        case 18
                            self.mag(2)=r_aux;
                        case 20
                            self.mag(3)=r_aux;
                        case 22
                            self.tspt=r_aux;
                        case 24
                            self.pres=r_aux;
                        case 26
                            self.trash=r_aux;
                        case 28
                            self.hum=r_aux;
                        case 30
                            self.temp2=r_aux;
                        case 32
                            self.temp1=r_aux;
                    end
                end
                self.tssnd=out(33)+out(34)*256;
                self.sound=out(35);
            end
            if isempty(out)==true
                error('ERROR: Wrong data format');
            end
        end
    end
end