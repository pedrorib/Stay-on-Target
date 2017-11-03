classdef agm < handle
    %agm - Fetches accelerometer, gyroscope and magnetometer data from ST
    %sensor and returns these as array elements of this class
    
    properties
        tstamp %Bluetooth timestamp
        acc %ST sensor output for accelerometer (as [X, Y, Z])
        gyr %ST sensor output for gyroscope (as [X, Y, Z])
        mag %ST sensor output for magnetometer (as [X, Y, Z])
    end
    properties(Access = private)
        RPi
        BT
    end
    methods
        function out = agm(pihandle,BTAddress)
            %Initializes connection to the ST sensor through the raspberry pi.
            %pt(pihandle,BTAddress):
            %pihandle - handle for raspberry pi, returned by raspi function
            %BTAddress - ST sensor MAC Address
            out.RPi=pihandle;
            out.BT=BTAddress;
            out.acc=[0 0 0];
            out.gyr=[0 0 0];
            out.mag=[0 0 0];
            out.tstamp=0;
            system(pihandle,['. ~/pasta_bt/conv_dados.sh ',BTAddress,' 0x0012 0x0011']);
        end
            
    end
    methods
        function self = update(self)
            %Updates the values of this class with new sensor data.
            %Should be executed without argument as <class>.update
            out=[];
            wd=0;
            while isempty(out) == true
                wd=wd+1;
                out=sscanf(system(self.RPi,['. ~/pasta_bt/conv_dados.sh ',self.BT,' 0x0012 0x0011']),'%x');
                if wd > 5
                    pause(1);
                    out=sscanf(system(self.RPi,['. ~/pasta_bt/conv_dados.sh ',self.BT,' 0x000f 0x000e']),'%x');
                    wd=0;
                end
            end
            for i=2:2:20
                aux1=out(i-1);
                aux2=out(i);
                r_aux=aux1+aux2*256;
                if r_aux > 32768
                    r_aux=r_aux-65536;
                end
                
                switch i
                    case 2
                        self.tstamp=r_aux;
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
                    otherwise
                        disp('Wrong data format from RPi.')
                        exit
                end
                
            end
        end
        
    end
    
end

