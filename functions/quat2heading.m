function heading = quat2heading(quat)
%% Gives the heading of a quaternion in radians    
% from: https://robotics.stackexchange.com/questions/16471/get-yaw-from-quaternion
    w = quat(:,1);
    x = quat(:,2);
    y = quat(:,3);
    z = quat(:,4); 
    heading = atan2(2.0 .* (w .* z + x .* y), w .* w + x .* x - y .* y - z .* z);
end