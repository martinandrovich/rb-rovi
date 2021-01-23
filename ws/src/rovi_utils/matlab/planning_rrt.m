close all; clear; clc;
run("rovi_common.m");

OPERATION = "pick";
METHOD = "RRTConnect";
POSE = "pose_0";
EXPERIMENT = "20210120_221926";

DIR_EXPERIMENT = DIR_DATA + "/planning_rrt/" + EXPERIMENT + "/" + OPERATION + "/" + METHOD + "/" + POSE;
% DIR_EXPERIMENT = DIR_DATA + "/planning_rrt/20210105_200351";

%% histograms

plan = readmatrix(DIR_EXPERIMENT + "/plan.csv");

% planning time
figure("Position", [0 0 350 500])
histogram(plan(:, 2), 20)
pbaspect([1 0.7 1])
xlabel("Time [ms]")
ylabel("Count")

export_fig(DIR_IMGS + "/planning/rrt-pick-plan-time-hist.pdf", "-painters")

% trajectory duration
figure("Position", [0 0 350 500])
histogram(plan(:, 3), 20)
pbaspect([1 0.7 1])
xlabel("Time [s]")
ylabel("Count")

export_fig(DIR_IMGS + "/planning/rrt-pick-traj-dur-hist.pdf", "-painters")

%% trajectory plot

% for transparency
set(groot, "DefaultFigureRenderer", "opengl");

figure
img = imread(DIR_IMGS + "/planning/rrt-traj-bg.png");
ax1 = axes();
imshow(img, "Parent", ax1);
ax2 = axes("Color", "none");
hold on

for i = 0:49
    traj = readmatrix(DIR_EXPERIMENT + "/traj" + i + ".csv");
    plot3(traj(:, 4), traj(:, 8), traj(:, 12), "LineWidth", 1, "Color", [0 207/255 255/255 0.2])
end

plot3(traj(1, 4), traj(1, 8), traj(1, 12), "O", "MarkerFaceColor", COL_ORANGE)
plot3(traj(end, 4), traj(end, 8), traj(end, 12), "O", "MarkerFaceColor", COL_MAGENTA)

% view(-167.6753, -10.1969)
% xlim([-0.1612, 0.9152])
% ylim([-0.0578, 1.9605])
% zlim([0.0697, 1.4152])

view(122.5521, -24.1283)
xlim([-0.3609 0.8792])
ylim([-0.7498 1.7303])
zlim([-0.2370 1.5346])

export_fig(DIR_IMGS + "/planning/rrt-traj-pick.pdf")