close all
clear all

% startup_rtb
robot = hexapod();

robot.plot();


%       T       x       y       z 
seg = [ 1       0       13.1    0 ;
        1       0       13.1    3 ;
        1       3       13.1    3 ;
        1       3       13.1    0 ;
        1       0       13.1    0 ; 
      ];


traj = mstraj(seg(:,2:4), [], seg(:,1)', seg(1,2:4), 0.01, 0.01);

[M N] = size(traj);
for i = 1:M
    robot.animate(traj(i, :));
end