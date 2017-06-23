function xtraj = postureSim( robot, pose , round, z_up)
%POSTURESIM Summary of this function goes here
%   Detailed explanation goes here
    xtraj = robot.pose2traj(pose, 5, 1, 5, z_up);
    xtraj = hexapod.traj2wavegait(xtraj, 5);

    T = xtraj{1}
    robot.footTipsPos = xtraj{1};
    
    a = Animate('image_posture');
    
    robot.plot();
    a.add()

    for i = 2:(31 * round)
        i = mod(i, 31) + 1
        T = xtraj{i}
        robot.animate(xtraj{i});
        a.add()
    end
end

