classdef hexapodBody < handle
    %HEXAPOD Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Constant)
        COXA = 7.5;
        FEMUR = 5.6;
        TIBIA = 7.5;
        
        LEG_BASE_X = 10;
        LEG_BASE_Y = 5;
        LEG_BASE_Z = hexapod.TIBIA;
        
        LEGS_BASE = [
            hexapod.LEG_BASE_X      -hexapod.LEG_BASE_Y   hexapod.LEG_BASE_Z;
                0                   -hexapod.LEG_BASE_Y   hexapod.LEG_BASE_Z;
            -hexapod.LEG_BASE_X     -hexapod.LEG_BASE_Y   hexapod.LEG_BASE_Z;
            hexapod.LEG_BASE_X      hexapod.LEG_BASE_Y    hexapod.LEG_BASE_Z;
                0                   hexapod.LEG_BASE_Y    hexapod.LEG_BASE_Z;
            -hexapod.LEG_BASE_X     hexapod.LEG_BASE_Y    hexapod.LEG_BASE_Z;
        ];
    end
    
    properties
        % Legs joint variable
        jointsVar;
        footTipsPos;
        
        zoffset;
        
        baseLegs;
        legs;
    end
    
    methods(Static)
        function r = leg()
            L1 = hexapod.COXA;
            L2 = hexapod.FEMUR;
            L3 = hexapod.TIBIA;
            % create the leg links based on DH parameters
            %                    theta   d     a  alpha
            links(1) = Link([    0       0    L1    pi/2]);
            links(2) = Link([    0       0    L2      0 ]);
            links(3) = Link([    0       0    L3      0 ]);

            r = SerialLink(links, 'name', 'leg', ...
                'base', transl(0, 0, L3), ...
                'offset', [-pi/2 0 -pi/2]);
        end
        
        function r = initBody6l()
            r_tmp(6,1) = SerialLink;
            for i = 1:6
                xb = hexapod.LEGS_BASE(i, 1);
                yb = hexapod.LEGS_BASE(i, 2);
                zb = hexapod.LEGS_BASE(i, 3);
                
                bl = Link([0 0 sqrt(xb^2 + yb^2) 0]);
                if xb < 0
                    bl.offset = pi + atan(yb/xb);
                else
                    bl.offset = atan(yb/xb);
                end
                r_tmp(i) = SerialLink(bl, ...
                    'name', strcat('base', int2str(i)), ...
                    'base', transl([0 0 zb]));
                r_tmp(i).qlim(1,:) = [0 0];
            end
            r = r_tmp;
        end
        
        function r = initLegs6l()
            r_tmp(6,1) = SerialLink;
            for i = 1:6
                xb = hexapod.LEGS_BASE(i, 1);
                yb = hexapod.LEGS_BASE(i, 2);
                offs = atan(xb/yb) + pi/2; 
                L = hexapod.leg();
                r_tmp(i) = SerialLink(L, ...
                    'name', strcat('leg', int2str(i)), ...
                    'offset', L.offset+[offs 0 0]);
            end
            r = r_tmp;
        end
        
        function r = getTransl(tran3d, rot3d)
            r = transl(tran3d) * trotx(rot3d(1)) ...
                * troty(rot3d(2)) * trotz(rot3d(3));
        end
    end
    
	methods        
        function obj = hexapod()
            obj.zoffset = hexapod.LEG_BASE_Z;
            obj.jointsVar = zeros(6,3);
            obj.footTipsPos = zeros(6,3);
            
            obj.legs = hexapod.initLegs6l();
            obj.baseLegs = hexapod.initBody6l();
            for i = 1:6
                obj.baseLegs(i).base = hexapod.getTransl([0 0 obj.zoffset], [0 0 0]);
                obj.legs(i).base = obj.baseLegs(i).fkine(0);
                obj.footTipsPos(i, :) = tform2trvec(obj.legs(i).fkine(obj.jointsVar(i, :)));
            end
        end
        
        function setBodyTransl(obj, v6d)
            v6d(3) = v6d(3) + obj.zoffset;
            v6d(4:6) = deg2rad(v6d(4:6));
            for i = 1:6
                obj.baseLegs(i).base = hexapod.getTransl(v6d(1:3), v6d(4:6));
            end
            obj.updateLegsBase();
            obj.animateBody();
        end
        
        function updateLegsBase(obj)
            for i = 1:6
                v_old = tform2trvec(obj.legs(i).fkine(obj.jointsVar(i,:)));
                obj.legs(i).base = obj.baseLegs(i).fkine(0);
                obj.jointsVar(i,:) = obj.legs(i).ikine(transl(v_old), [0 0 0], [1 1 1 0 0 0]);
            end
        end
        
        function plot(obj)
            plotopt = {'nobase', 'nowrist', 'noname', ...
                       'delay', 0.1, ...
                       'noshading', 'floorlevel', 0};
            hold on
            axis([-30 30 -30 30 -5 20]);
            for i = 1:6
                obj.legs(i).plot(obj.jointsVar(i,:), plotopt{:});
                obj.baseLegs(i).plot(0, plotopt{:});
            end
            hold off
        end
        
        function animateBody(obj)
            for i = 1:6
                obj.legs(i).animate(obj.jointsVar(i,:));
                obj.baseLegs(i).animate(0);
            end
        end
    end
end

