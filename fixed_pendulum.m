%% For displaying and generating csv files from stationary pendulum payload tests

% When the UI appears, choose the appropriate .ulg file

disp('start')

%% Functions
quat_rot_vect = @(vect, quat) quatrotate(quatinv(quat), vect); % Rotates vector by quaternion % built in "quatrotate" rotates the coordinate frame, not the vector, therefore use inverse in function (https://www.mathworks.com/matlabcentral/answers/465053-rotation-order-of-quatrotate)

%% Load topics from csv into matrix
load_csv_again = 1;
if load_csv_again
    [ulog_name,csv_folder] = uigetfile('./logs/*.ulg', 'Choose ulog file to access') % GUI to choose ulog file. % [ulog filename, path to folder with csv files]
    ulog_name = erase(ulog_name, '.ulg'); % remove file extention

    adc_report = readmatrix(strcat(csv_folder, ulog_name, '_', 'adc_report', '_0.csv'));
    estimator_status = readmatrix(strcat(csv_folder, ulog_name, '_', 'estimator_status', '_0.csv'));

    disp('loaded csv files')
end

%% Time matching
% have common time series so no extrapolation occurs between timeseries

adc_time = adc_report(:,1)./1e6; % Timestamp of adc_report in seconds
combo_time = adc_time; % Use combo_time for when code is copied between scripts for consistency

%% Joystick attitude

% Convertion from adc value to radians
green_pot_line_fit = [ 0.038980944549164 -37.789860132384199]; % degrees linefit for polyval from calibration of pot connected to green wire
blue_pot_line_fit  = [ 0.018768173769117 -37.181837589261562];

offset_y = -0.0685; %-0.033242678592147; % [degrees] Offset calculated afterwards
offset_x = -0.013; % -0.037739964002411; % [degrees] Offset calculated afterwards

green_adc2angle = @(adc) deg2rad(polyval(green_pot_line_fit, adc)) - offset_y; % Convert green adc value to angle [rad]
blue_adc2angle  = @(adc) deg2rad(polyval(blue_pot_line_fit,  adc)) - offset_x; % Convert green adc value to angle [rad]

% Define payload angle as euler angle, convention: 'ZYX'. 
% joystick x-axis connected to drone.
% joystick y-axis connected to x-axis
% z-axis does not matter
j_y = green_adc2angle(adc_report(:,3+4)); % [radians] Euler y angle of joystick. (side to side) (3+ to convert Channel_ID of adc_report to index)
j_x = blue_adc2angle(adc_report(:,3+10)); % [radians] Euler x angle of joystick. (forwards backwards)
j_z = zeros(size(j_y)); % No z angle

joy_euler = [j_z, j_y, j_x]; %???debug Euler angles of joystick % MATLAB euler format is [z, y, x]

joy_quat    = eul2quat(joy_euler, 'ZYX');
joy_quat_ts = timeseries(joy_quat, adc_time, 'Name', 'Attitude'); % Time series of euler angles of drone

disp('joystick time series')

%% Payload attitude

payload_vector  = quat_rot_vect([0 0 1], joy_quat); % unit vector representing direction of payload. Rotate neutral hanging payload by joystick angle, then attitude. % "quatrotate" rotates the coordinate frame, not the vector, therefore use inverse in function (https://www.mathworks.com/matlabcentral/answers/465053-rotation-order-of-quatrotate)

payload_vector_angle_x = -atan2(payload_vector(:,2), payload_vector(:,3)); % [radians] absolute angle of payload vector from z axis, about the x axis, projected on yz plane. NOT euler angle. negative, becasue +y gives negative rotation about x
payload_vector_angle_y =  atan2(payload_vector(:,1), payload_vector(:,3)); % [radians] absolute angle of payload vector from z axis, about the y axis, projected on xz plane. NOT euler angle

payload_vector_angles = [payload_vector_angle_x, payload_vector_angle_y]; % [radians] [x, y] absolute angle of payload vector. NOT euler angles

%% Plots
close all;

figure;
plot(combo_time, rad2deg(payload_vector_angles(:,1)));
legend('x');
title('payload_vector_angles');

figure;
plot(combo_time, [j_x, j_y]);
legend('jx', 'jy');
title('euler angles');

%% Write data to csv files
% 
data_table = array2table([combo_time, rad2deg(payload_vector_angles(:,1))]);

data_table.Properties.VariableNames = {'t', 'angle'};
writetable(data_table, ['csv/', csv_folder(end-11:end-1), '.csv'])

disp('csv generated')

