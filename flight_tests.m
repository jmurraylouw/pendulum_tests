%% Project payload angles to absolute coordinate frame 

disp('start')
%% Functions
quat_rot_vect = @(vect, quat) quatrotate(quatinv(quat), vect); % Rotates vector by quaternion % built in "quatrotate" rotates the coordinate frame, not the vector, therefore use inverse in function (https://www.mathworks.com/matlabcentral/answers/465053-rotation-order-of-quatrotate)

%% Load topics from csv into matrix
load_csv_again = 1;
if load_csv_again
    current_dir = pwd;
    [ulog_name,csv_folder] = uigetfile([current_dir, '/*.ulg'], 'Choose ulog file to access') % GUI to choose ulog file. % [ulog filename, path to folder with csv files]
    ulog_name = erase(ulog_name, '.ulg'); % remove file extention

    adc_report = readmatrix(strcat(csv_folder, ulog_name, '_', 'adc_report', '_0.csv'));
    estimator_status = readmatrix(strcat(csv_folder, ulog_name, '_', 'estimator_status', '_0.csv'));

    disp('loaded csv files')
end

%% Time matching
% have common time series so no extrapolation occurs between timeseries

state_time = estimator_status(:,1)./1e6; % Timestamp of state data in seconds
adc_time = adc_report(:,1)./1e6; % Timestamp of adc_report in seconds

max_time = min([max(adc_time), max(state_time)]); % get max value in time array that overlaps in both timeseries
min_time = max([min(adc_time), min(state_time)]); % get min value in time array that overlaps in both timeseries
basis_time = state_time; % Use state_time as basis.
combo_time = basis_time(  (min_time <= basis_time)  &  (basis_time <= max_time)  ); % Remove values outside overlapping range

disp("time matching done")

%% Attitude

uav_quat    = estimator_status(:, (0:3) + 2); % Quaternions of uav +2 to use index from https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html - "The index map for states[32] is as follows:"
uav_quat_ts = timeseries(uav_quat, state_time, 'Name', 'Attitude'); % Time series of euler angles of drone
uav_quat_ts = resample(uav_quat_ts, combo_time, 'linear'); % Resample for matching time sequence
uav_quat    = uav_quat_ts.Data; % Data from resampled timeseries

%% Position

pos    = estimator_status(:, (7:9) + 2); % position of uav [x,y,z]
pos_ts = timeseries(pos, state_time, 'Name', 'Position'); % Time series
pos_ts = resample(pos_ts, combo_time, 'linear'); % Resample for matching time sequence
pos    = pos_ts.Data; % Data from resampled timeseries

%% Velocity

vel    = estimator_status(:, (4:6) + 2); % position of uav [x,y,z]
vel_ts = timeseries(vel, state_time, 'Name', 'Velocity'); % Time series
vel_ts = resample(vel_ts, combo_time, 'linear'); % Resample for matching time sequence
vel    = vel_ts.Data; % Data from resampled timeseries

%% Remove Z of uav attitude

heading = quat2heading(uav_quat);
quat_inv_heading = quatinv(eul2quat([heading, zeros(size(heading)), zeros(size(heading))])); % inverse of quat of heading 

uav_quat = quatmultiply(quat_inv_heading, uav_quat); % Remove heading

%% UAV into vector form
uav_vector  = quat_rot_vect([0 0 1], uav_quat); % unit vector representing direction of payload. Rotate neutral hanging payload by joystick angle, then attitude. % "quatrotate" rotates the coordinate frame, not the vector, therefore use inverse in function (https://www.mathworks.com/matlabcentral/answers/465053-rotation-order-of-quatrotate)
uav_vector_angle_x = -atan2(uav_vector(:,2), uav_vector(:,3)); % [radians] absolute angle of payload vector from z axis, about the x axis, projected on yz plane. NOT euler angle. negative, becasue +y gives negative rotation about x
uav_vector_angle_y =  atan2(uav_vector(:,1), uav_vector(:,3)); % [radians] absolute angle of payload vector from z axis, about the y axis, projected on xz plane. NOT euler angle

uav_vector_angles = [uav_vector_angle_x, uav_vector_angle_y]; % [radians] [x, y] absolute angle of payload vector. NOT euler angles

disp('state time series')

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

joy_quat_ts = resample(joy_quat_ts, combo_time, 'linear'); % Resample for matching time sequence
joy_quat    = joy_quat_ts.Data; % Data from resampled timeseries

disp('joystick time series')

%% Payload attitude

payload_abs_rot = quatmultiply(uav_quat, joy_quat); % Attitude of payload in World frame. First joystick rotation. Then UAV attitude rotation

payload_vector  = quat_rot_vect([0 0 1], payload_abs_rot); % unit vector representing direction of payload. Rotate neutral hanging payload by joystick angle, then attitude. % "quatrotate" rotates the coordinate frame, not the vector, therefore use inverse in function (https://www.mathworks.com/matlabcentral/answers/465053-rotation-order-of-quatrotate)

payload_vector_angle_x = -atan2(payload_vector(:,2), payload_vector(:,3)); % [radians] absolute angle of payload vector from z axis, about the x axis, projected on yz plane. NOT euler angle. negative, becasue +y gives negative rotation about x
payload_vector_angle_y =  atan2(payload_vector(:,1), payload_vector(:,3)); % [radians] absolute angle of payload vector from z axis, about the y axis, projected on xz plane. NOT euler angle

payload_vector_angles = [payload_vector_angle_x, payload_vector_angle_y]; % [radians] [x, y] absolute angle of payload vector. NOT euler angles

%% Plots
close all;

figure;
plot(combo_time, rad2deg(heading));
title('heading');

% figure;
% plot(combo_time, (uav_vector));
% legend('x', 'y', 'z');
% title('uav_vector');

figure;
plot(combo_time, rad2deg(uav_vector_angles));
legend('x', 'y', 'z');
title('uav vector angles');

figure;
plot(combo_time, rad2deg(payload_vector_angles(:,2)));
legend('x', 'y');
title('payload vector angles');

%% Position and velocity

figure
plot(combo_time, pos(:,1))
title('position')
legend('x', 'y', 'z')

figure
plot(combo_time, vel(:,1))
title('velocity')
legend('x', 'y', 'z')

%%
figure;
plot(adc_time, (j_x));
title('j_x');

figure;
plot(adc_time, (j_y));
title('j_y');
% 
% %%
% figure;
% title('euler angles');
% plot(uav_euler_ts);
% legend('Z', 'Y', 'X');

disp('plotted')


%% Write data to csv files
% Current folder should be in project folder for this to work
data_table = array2table([combo_time, rad2deg(payload_vector_angles(:,1)), pos(:,1), vel(:,1)]);

data_table.Properties.VariableNames = {'t', 'angle', 'x_pos', 'x_vel'};
csv_index = strfind(csv_folder,'/');
csv_name = csv_folder( csv_index(end-1)+1 : end-1 )
writetable(data_table, ['csv/flights/', csv_name, '.csv'])

disp('csv generated')

