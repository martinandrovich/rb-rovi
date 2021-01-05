close all; clear; clc;
run("rovi_common.m");

DIR_EXPERIMENT = DIR_DATA + "/interpolation_kdl/20201230_222117";

% load data
traj_lin = readmatrix(DIR_EXPERIMENT + "/traj_lin.csv");
traj_par = readmatrix(DIR_EXPERIMENT + "/traj_par.csv");
traj_dur = readmatrix(DIR_EXPERIMENT + "/dur.txt");

%% linear interpolation

% compute variables
data = traj_lin;

dur = traj_dur(1);
dt = 0.01;
t = 0:dt:dur;

x = data(:, 4);
y = data(:, 8);
z = data(:, 12);

xdot = gradient(x(:)) ./ gradient(t(:));
ydot = gradient(y(:)) ./ gradient(t(:));

% plot
% figure("Position", [0 0 1500 500]);
% sgtitle("Linear interpolation");
% 
% subplot(1,3,1)
% title("Position")
% hold on
% plot(t, x)
% plot(t, y)
% pbaspect([1 1 1])
% legend(["x", "y"]);
% xlabel("Time [s]"); ylabel("Position [m]");
% 
% subplot(1,3,2)
% title("Velocity")
% pbaspect([1 1 1])
% hold on
% plot(t, xdot)
% plot(t, ydot)
% legend(["x", "y"]);
% xlabel("Time [s]"); ylabel("Velocity [m/s]");
% 
% subplot(1,3,3)
% plot3(x, y, z)
% title("Trajectory")
% pbaspect([1 1 1])
% grid on
% xlabel("x"); ylabel("y"); zlabel("z");
% view(-30, 15);
% 
% export_fig(DIR_IMGS + "/traj-lin-demo.pdf")

close all;

figure
hold on
plot(t, x)
plot(t, y)
pbaspect([1 1 1])
legend(["x", "y"]);
xlabel("Time [s]"); ylabel("Position [m]");

export_fig(DIR_IMGS + "/traj-lin-demo-pos.pdf")

figure
pbaspect([1 1 1])
hold on
plot(t, xdot)
plot(t, ydot)
legend(["x", "y"]);
xlabel("Time [s]"); ylabel("Velocity [m/s]");

export_fig(DIR_IMGS + "/traj-lin-demo-vel.pdf")

figure
plot3(x, y, z)
pbaspect([1 1 1])
grid on
xlabel("x"); ylabel("y"); zlabel("z");
view(-30, 15);

export_fig(DIR_IMGS + "/traj-lin-demo-traj.pdf")

%% parabolic interpolation

% compute variables
data = traj_par;

dur = traj_dur(2);
dt = 0.01;
t = 0:dt:dur;

x = data(:, 4);
y = data(:, 8);
z = data(:, 12);

xdot = gradient(x(:)) ./ gradient(t(:));
ydot = gradient(y(:)) ./ gradient(t(:));

% plot

figure
hold on
plot(t, x)
plot(t, y)
pbaspect([1 1 1])
legend(["x", "y"]);
xlabel("Time [s]"); ylabel("Position [m]");

export_fig(DIR_IMGS + "/traj-par-demo-pos.pdf")

figure
pbaspect([1 1 1])
hold on
plot(t, xdot)
plot(t, ydot)
legend(["x", "y"]);
xlabel("Time [s]"); ylabel("Velocity [m/s]");

export_fig(DIR_IMGS + "/traj-par-demo-vel.pdf")

figure
plot3(x, y, z)
pbaspect([1 1 1])
grid on
xlabel("x"); ylabel("y"); zlabel("z");
view(-30, 15);

export_fig(DIR_IMGS + "/traj-par-demo-traj.pdf")