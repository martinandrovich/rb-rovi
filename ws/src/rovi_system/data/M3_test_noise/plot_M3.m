A = readmatrix('rovi_pose.csv')
z_1 = find(A(:,1) == 3*3);
z_2 = find(A(:,1) == 6*6);
z_3 = find(A(:,1) == 9*9);
z_4 = find(A(:,1) == 12*12);
z_5 = find(A(:,1) == 15*15);

run("rovi_common.m");

figure(1)
do_it = scatter_plot_experiment(A, z_1)
%export_fig("M3_3_scatter.pdf")

figure(2)
now = plot_quat_experimenet(A,z_1)
%export_fig("M3_3_ori.pdf")


%%
figure(3)
do_it = scatter_plot_experiment(A, z_2)
%export_fig("M3_6_scatter.pdf")

figure(4)
n0w = plot_quat_experimenet(A, z_2)
%export_fig("M3_6_ori.pdf")


%%
figure(5)
do_it = scatter_plot_experiment(A, z_3)
%export_fig("M3_9_scatter.pdf")

figure(6)
n0w = plot_quat_experimenet(A, z_3)
%export_fig("M3_9_ori.pdf")

%%

figure(7)
do_it = scatter_plot_experiment(A, z_4)
%export_fig("M3_12_scatter.pdf")

figure(8)
now = plot_quat_experimenet(A, z_4)
%export_fig("M3_12_ori.pdf")

%%

figure(9)
scatter_plot_experiment(A, z_5)
export_fig("M3_12_scatter.pdf")

figure(10)
plot_quat_experimenet(A, z_5)
export_fig("M3_12_ori.pdf")



function ave = scatter_plot_experiment(A, idx)
    act_x = A(idx,2);
    act_y = A(idx,3);
    guess_x = A(idx,9);
    guess_y = A(idx,10);
    scatter(100*act_x-100*guess_x, 100*act_y-100*guess_y);
    xlabel('x [cm]')
    ylabel('y [cm]')
    set(gcf, 'Position', [0 0 500 500]);
    ytickformat('%.2f');
    xtickformat('%.2f');
    legend('Error');
    axis([-2 2 -2 2])
    L2 = [100*act_x-100*guess_x 100*act_y-100*guess_y];
    L2 = sqrt(L2(:,1).^2 + L2(:,2).^2);
    ave = length(find(L2 > 1.1)) / length(L2)
end

function [out] = plot_quat_experimenet(A, idx)
    Q1 = A(idx,5:8);
    Q2 = A(idx,12:15);
    Q3 = quatmultiply(quatconj(Q1),Q2);
    angle = 2 * atan2(Q3(:,2:4),Q3(:,1)) * 180/pi;
    idx = find(abs(angle) < 25);
    histogram(abs(angle(idx)));
    xlabel('Degrees [*]');
    set(gcf, 'Position', [0 0 500 500]);
    ytickformat('%.2f');
    xtickformat('%.2f');
    legend('Error in Orientation');
    axis([0 30 0 200]);
    find(angle(:,1) < 10);
    out = length(find(angle(:,1) < 10)) / length(angle(:,1));
    length(angle(:,1))
end