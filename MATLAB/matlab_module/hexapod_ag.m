classdef hexapod_ag < handle
    properties
        % Legs joint variable
        robot;
    end
    
    methods        
        function obj = hexapod_ag()
            obj.robot = hexapod();
        end
        
        function r = legIk(obj, x, y, z)
            C = hexapod.COXA;
            F = hexapod.FEMUR;
            T = hexapod.TIBIA;
            z_off = obj.robot.zoffset;
            
            alpha = atan(x/y);
            L1 = sqrt(x^2 + y^2);
            L = sqrt((L1 - C)^2 + (z_off - z)^2);
            beta = acos((F^2 + L^2 - T^2)/(2*L*F)) - atan((z_off - z)/(L1 - C));
            gramma = acos((T^2 + F^2 - L^2)/(2*T*F)) - pi/2;
            
            r = [alpha beta gramma];
        end
    end
end
