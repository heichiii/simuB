clear;
yaw_in=0;
pitch_in=0;
roll_in=0;
R06 =[  cos(pitch_in)*cos(yaw_in),                                              -cos(pitch_in)*sin(yaw_in),                 sin(pitch_in);
    cos(roll_in)*sin(yaw_in) + cos(yaw_in)*sin(pitch_in)*sin(roll_in), cos(roll_in)*cos(yaw_in) - sin(pitch_in)*sin(roll_in)*sin(yaw_in), -cos(pitch_in)*sin(roll_in);
    sin(roll_in)*sin(yaw_in) - cos(roll_in)*cos(yaw_in)*sin(pitch_in), cos(yaw_in)*sin(roll_in) + cos(roll_in)*sin(pitch_in)*sin(yaw_in),  cos(pitch_in)*cos(roll_in)]
% R06 =[  cos(pitch_in)*cos(yaw_in),                                              -cos(pitch_in)*sin(yaw_in),                 sin(pitch_in);


