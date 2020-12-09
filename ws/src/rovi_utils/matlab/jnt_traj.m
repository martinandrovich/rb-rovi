close all; clear; clc;
run("rovi_common.m");

q = readmatrix(DIR_DATA + "/test/traj_rrt_q.csv");
qdot = readmatrix(DIR_DATA + "/test/traj_rrt_qdot.csv");

figure()
for i = 1:6
    subplot(2,3,i)
    hold on, grid on
    plot(q(:, i))
    plot(qdot(:, i))
    pbaspect([1 1 1])
    title("Joint " + i)
    xlabel("Time [ms]")
    legend(["$q$", "$\dot{q}$"], "Interpreter", "latex")
end

export_fig(DIR_IMGS + "/jnt_traj.pdf")