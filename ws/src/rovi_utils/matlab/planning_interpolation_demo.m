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

close all;

figure
hold on
plot(t, x, "LineWidth", 3)
plot(t, y, "LineWidth", 3)
pbaspect([1 1 1])
legend(["x", "y"]);
xlabel("Time [s]"); ylabel("Position [m]");

export_fig(DIR_IMGS + "/planning/traj-lin-demo-pos.pdf")

figure
pbaspect([1 1 1])
hold on
plot(t, xdot, "LineWidth", 3)
plot(t, ydot, "LineWidth", 3)
legend(["x", "y"]);
xlabel("Time [s]"); ylabel("Velocity [m/s]");

export_fig(DIR_IMGS + "/planning/traj-lin-demo-vel.pdf")

figure
plot3(x, y, z, "LineWidth", 3)
pbaspect([1 1 1])
grid on
xlabel("x"); ylabel("y"); zlabel("z");
view(-30, 15);

export_fig(DIR_IMGS + "/planning/traj-lin-demo-traj.pdf")

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
plot(t, x, "LineWidth", 3)
plot(t, y, "LineWidth", 3)
pbaspect([1 1 1])
legend(["x", "y"]);
xlabel("Time [s]"); ylabel("Position [m]");

export_fig(DIR_IMGS + "/planning/traj-par-demo-pos.pdf")

figure
pbaspect([1 1 1])
hold on
plot(t, xdot, "LineWidth", 3)
plot(t, ydot, "LineWidth", 3)
legend(["x", "y"]);
xlabel("Time [s]"); ylabel("Velocity [m/s]");

export_fig(DIR_IMGS + "/planning/traj-par-demo-vel.pdf")

figure
plot3(x, y, z, "LineWidth", 3)
pbaspect([1 1 1])
grid on
xlabel("x"); ylabel("y"); zlabel("z");
view(-30, 15);

export_fig(DIR_IMGS + "/planning/traj-par-demo-traj.pdf")