function writePostureToFile( num_id, posture )
%writePostureToFile Summary of this function goes here
%   Detailed explanation goes here
    fileID = fopen(strcat('.\posture\', 'POSE', num2str(num_id), '.DAT'), 'w');
    for i = 1:6
        fwrite(fileID, posture(i, 1), 'int32');
        fwrite(fileID, posture(i, [2:7]), 'float32');
    end
    fclose(fileID);
end

