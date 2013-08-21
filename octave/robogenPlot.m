%% Octave-compatible script to plot robot trajectory and obstacles
% TODO test with Matlab
clear all;
close all;

%% Plot trajectory
figure();
traj = load("trajectoryLog.txt");
plot(traj(:,1),traj(:,2));

%% Plot obstacles
hold on;
obs = load("obstacles.txt");
for i=1:rows(obs)
	o_r = obs(i,:);
	o_x = [o_r(1)-o_r(3)/2, o_r(1)+o_r(3)/2, o_r(1)+o_r(3)/2, o_r(1)-o_r(3)/2, ...
					o_r(1)-o_r(3)./2];
	o_y = [o_r(2)-o_r(4)/2,	o_r(2)-o_r(4)/2, o_r(2)+o_r(4)/2,	o_r(2)+o_r(4)/2, ...
					o_r(2)-o_r(4)/2];
	% because the ubuntu-default octave won't support the rectangle() function
	plot(o_x,o_y);
end
title("Robot trajectory");
hold off;

%% Plot sensor values
figure();
sens = load("sensorLog.txt");
plot(1:rows(sens), sens);
axis([0 rows(sens) -1.1 1.1]);
title("Sensor history");

%% Plot motor values
figure();
mot = load("motorLog.txt");
plot(1:rows(mot), mot);
axis([0 rows(mot) -0.1 1.1]);
title("Motor history");