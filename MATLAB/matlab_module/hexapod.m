classdef hexapod < handle
    %HEXAPOD Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Constant)
        COXA = 7.5;
        FEMUR = 5.6;
        TIBIA = 7.5;
        
        LEG_BASE_X = 15.75;
        LEG_BASE_Y = 4.375;
        LEG_BASE_Z = 7.5;
        
        LEG_INIT_FT = [0 13.1 0];
        
        LEGS_BASE = [
            hexapod.LEG_BASE_X      hexapod.LEG_BASE_Y    hexapod.LEG_BASE_Z;
                0                   hexapod.LEG_BASE_Y    hexapod.LEG_BASE_Z;
            -hexapod.LEG_BASE_X     hexapod.LEG_BASE_Y    hexapod.LEG_BASE_Z;
            hexapod.LEG_BASE_X      -hexapod.LEG_BASE_Y   hexapod.LEG_BASE_Z;
                0                   -hexapod.LEG_BASE_Y   hexapod.LEG_BASE_Z;
            -hexapod.LEG_BASE_X     -hexapod.LEG_BASE_Y   hexapod.LEG_BASE_Z;
        ];
    end
    
    properties
        % Legs joint variable
        footTipsPos;
        
        zoffset;
        
        legs;
    end
    
    methods(Static)
        function r = leg(z_off)
            L1 = hexapod.COXA;
            L2 = hexapod.FEMUR;
            L3 = hexapod.TIBIA;
            % create the leg links based on DH parameters
            %                    theta   d     a  alpha
            links(1) = Link([    0       0    L1    pi/2]);
            links(2) = Link([    0       0    L2      0 ]);
            links(3) = Link([    0       0    L3      0 ]);

            r = SerialLink(links, 'name', 'leg', ...
                'base', transl(0, 0, z_off), ...
                'offset', [pi/2 0 -pi/2]);
        end
        
        function r = initLegs6l(z_off)
            r_tmp(6,1) = SerialLink;
            for i = 1:6
                xb = hexapod.LEGS_BASE(i, 1);
                yb = hexapod.LEGS_BASE(i, 2);
                rot = trotz(0);
                if yb < 0
                    rot = trotz(pi);
                end
                L = hexapod.leg(z_off);
                r_tmp(i) = SerialLink(L, ...
                    'base', transl(xb, yb, z_off)*rot, ...
                    'name', strcat('leg', int2str(i)));
            end
            r = r_tmp;
        end
        
        function r = legIk(ft, z_off)
            x = ft(1);
            y = ft(2);
            z = ft(3);
            C = hexapod.COXA;
            F = hexapod.FEMUR;
            T = hexapod.TIBIA;
            
            alpha = atan(x/y);
            L1 = sqrt(x^2 + y^2);
            L = sqrt((L1 - C)^2 + (z_off - z)^2);
            beta = acos((F^2 + L^2 - T^2)/(2*L*F)) - atan((z_off - z)/(L1 - C));
            gramma = acos((T^2 + F^2 - L^2)/(2*T*F)) - pi/2;
            
            r = [alpha beta gramma];
        end
    end
    
	methods        
        function obj = hexapod()
            obj.zoffset = hexapod.LEG_BASE_Z;
            obj.footTipsPos = repmat(hexapod.LEG_INIT_FT, 6, 1);
            obj.legs = hexapod.initLegs6l(obj.zoffset);
        end
        
        function r = legIkForLeg(obj, legId)
            q = hexapod.legIk(obj.footTipsPos(legId,:), obj.zoffset);
            
            if legId < 4
                q(1) = -q(1);
            end
            r = q;
        end
        
        function plot(obj)
            plotopt = {'nobase', 'nowrist', 'noname', ...
                       'delay', 0.1, ...
                       'noshading', 'floorlevel', 0};
            
            view(3)
            axis([-30 30 -30 30 -5 20]);
            hold on
            
            patch_vert = [0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1];
            patch_fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
            patch_scale = [hexapod.LEG_BASE_X*2 + 6 ...
                    hexapod.LEG_BASE_Y*2 ...
                    hexapod.LEG_BASE_Z*0.5];
            patch_move = [-hexapod.LEG_BASE_X - 3 ...
                    -hexapod.LEG_BASE_Y ...
                    hexapod.LEG_BASE_Z*0.75];
            
            patch_vert = (patch_vert .* repmat(patch_scale,8,1)) + repmat(patch_move,8,1);
            
            patch('Vertices', patch_vert, 'Faces', patch_fac, 'FaceVertexCData', [1:6]', 'FaceColor', 'flat');
            for i = 1:6
                q = obj.legIkForLeg(i);
                obj.legs(i).plot(q, plotopt{:});
            end
            hold off
        end
        
        function animate(obj, ft)
            for i = 1:6
                obj.footTipsPos(i,:) = ft;
                q = obj.legIkForLeg(i)
                obj.legs(i).animate(q);
            end
        end
    end
end

