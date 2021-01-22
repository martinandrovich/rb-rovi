A = readmatrix('rovi_pose.csv')
run("rovi_common.m");
z_0 = find(A(:,1) == 0*0)

figure(1)
L2 = scatter_plot_experiment(A, z_0)
%export_fig("ape.pdf")

figure(2)
angle = plot_quat_experimenet(A,z_0)
%export_fig("orientation.pdf")
find(angle(:,1) < 10)
length(find(angle(:,1) < 10))
length(angle(:,1))

function L2 = scatter_plot_experiment(A, idx)
    act_x = A(idx,2)
    act_y = A(idx,3)
    guess_x = A(idx,9)
    guess_y = A(idx,10)
    scatter(100*act_x - 100*guess_x, 100*act_y - 100*guess_y)
    xlabel('x [cm]')
    ylabel('y [cm]')
    set(gcf, 'Position', [0 0 500 500]);
    ytickformat('%.2f');
    xtickformat('%.2f');
    axis([-2 2 -2 2])
    legend('Error')
    
    L2 = [100*act_x - 100*guess_x 100*act_y - 100*guess_y]
    L2 = sqrt(L2(:,1).^2 + L2(:,2).^2)
    find(L2 > 1.1)
    
end

function [angle] = plot_quat_experimenet(A, idx)
    Q1 = A(idx,5:8)
    Q2 = A(idx,12:15)
    Q3 = quatmultiply(quatconj(Q1),Q2) 
    angle = 2 * atan2(Q3(idx,2:4),Q3(idx,1)) * 180/pi
    idx = find(abs(angle) < 25)
    histogram(abs(angle(idx)))
    xlabel('Degrees [*]')
    set(gcf, 'Position', [0 0 500 500]);
    ytickformat('%.2f');
    xtickformat('%.2f');
    legend('Error in Orientation')
    %axis([0.2 0.7 0.95 1.1]*100)
    
end
