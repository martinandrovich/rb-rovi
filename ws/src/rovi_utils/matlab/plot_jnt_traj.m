clear; close all; clc;

q = readmatrix("traj_rrt_q.csv");
qdot = readmatrix("traj_rrt_qdot.csv");

figure()
for i = 1:6
    subplot(2,3,i)
    hold on, grid on
    plot(q(:, i))
    plot(qdot(:, i))
    title("Joint " + i)
    xlabel("Time [ms]")
    legend(["$q$", "$\dot{q}$"], "Interpreter", "latex")
end