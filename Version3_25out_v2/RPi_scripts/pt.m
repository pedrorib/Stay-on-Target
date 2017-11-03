classdef pt < handle
    %pt - Fetches pressure and temperature from ST sensor and returns these
    %as elements of this class as scalar signed integers
    
    properties
        tstamp %Bluetooth timestamp
        temp %ST sensor output for temperature
        trash %Unused data
        pres %ST sensor output for pressure
        pres2
        hum
    end
    properties(Access = private)
        RPi
        BT;
    end
    
    methods
        function out = pt(pihandle,BTAddress)
            %Initializes connection to the ST sensor through the raspberry pi.
            %pt(pihandle,BTAddress):
            %pihandle - handle for raspberry pi, returned by raspi function
            %BTAddress - ST sensor MAC Address
            out.RPi=pihandle;
            out.BT=BTAddress;
            out.temp=0;
            out.pres=0;
            out.tstamp=0;
            out.hum=0;
            out.pres2=0;
            system(pihandle,['. ~/pasta_bt/conv_dados.sh  ',BTAddress,' 0x000f 0x000e']);
        end
        
    end
    methods
        function self = update(self)
            %Updates the values of this class with new sensor data.
            %Should be ran without arguments as <class>.update
            out=[];
            wd=0;
            while isempty(out) == true
                wd=wd+1;
                out=sscanf(system(self.RPi,['. ~/pasta_bt/conv_dados.sh ',self.BT,' 0x000f 0x000e']),'%x');
                if wd > 5
                    pause(1);
                    out=sscanf(system(self.RPi,['. ~/pasta_bt/conv_dados.sh ',self.BT,' 0x000f 0x000e']),'%x');
                    wd=0;
                end
            end
            for i=2:2:12
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
                        self.pres=r_aux;
                    case 6
                        self.trash=r_aux;
                    case 8
                        self.pres2=r_aux;
                    case 10
                        self.hum=r_aux;
                    case 12
                        self.temp=r_aux;
                end
            end
            
        end
    end
    
end

