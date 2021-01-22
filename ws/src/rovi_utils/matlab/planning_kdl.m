close all; clear; clc;
run("rovi_common.m");

DIR_EXPERIMENT = DIR_DATA + "/planning_kdl/20210105_203911";

% load data
traj_lin = readmatrix(DIR_EXPERIMENT + "/traj_lin.csv");
traj_par = readmatrix(DIR_EXPERIMENT + "/traj_par.csv");
traj_pts = readmatrix(DIR_EXPERIMENT + "/waypoints.csv");
plan_lin = readmatrix(DIR_EXPERIMENT + "/plan_lin.csv");
plan_par = readmatrix(DIR_EXPERIMENT + "/plan_par.csv");

% traj_dur = readmatrix(DIR_EXPERIMENT + "/dur.txt");

%% linear vs parabolic paths

% compute variables
data = traj_lin;

x = data(:, 4);
y = data(:, 8);
z = data(:, 12);

figure
hold on
plot3(x, y, z, "LineWidth", 3, "Color", COL_BLUE)
plot3(traj_pts(:, 4), traj_pts(:, 8), traj_pts(:, 12), "O", "MarkerFaceColor", COL_ORANGE)
pbaspect([1 1 1])
grid on
xlabel("x"); ylabel("y"); zlabel("z");
view(150, 25);

export_fig(DIR_IMGS + "/planning/kdl-traj-lin.pdf")

data = traj_par;

x = data(:, 4);
y = data(:, 8);
z = data(:, 12);

figure
plot3(x, y, z, "LineWidth", 3, "Color", COL_BLUE)
hold on
plot3(traj_pts(:, 4), traj_pts(:, 8), traj_pts(:, 12), "O", "MarkerFaceColor", COL_ORANGE)
pbaspect([1 1 1])
grid on
xlabel("x"); ylabel("y"); zlabel("z");
view(150, 25);

export_fig(DIR_IMGS + "/planning/kdl-traj-par.pdf")

%% histograms

figure("Position", [0 0 700 500])

subplot(1,2,1)
histogram(plan_lin(:, 2), 40, "FaceColor", COL_BLUE)
title("Linear interpolation")
mean(plan_lin(:, 2))
xlabel("Time [ms]")
ylabel("Count")
xlim([0 0.03])
pbaspect([1 0.7 1])
% yticklabels(yticks*100)

subplot(1,2,2)
histogram(plan_par(:, 2), 20, "FaceColor", COL_BLUE)
title("Parabolic interpolation")
mean(plan_lin(:, 2))
xlabel("Time [ms]")
ylabel("Count")
xlim([0 0.03])
pbaspect([1 0.7 1])
% yticklabels(yticks*100)

export_fig(DIR_IMGS + "/planning/kdl-plan-time.pdf", "-painters")
