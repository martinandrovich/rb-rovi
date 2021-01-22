function [l2norm, mu1, covar] = plot_experiment_3(noise, zero, A, name, fig)
    run("rovi_common.m");
    figure(fig(1))
    pos_zero = zero
    zero = [awgn(A(zero,5),noise,0) awgn(A(zero,6),noise,0)]
    l2norm = sqrt((zero(:,1) - A(pos_zero,2)).^2 + (zero(:,2) - A(pos_zero,3)).^2)
    scatter(zero(:,1), zero(:,2))
    xlabel('$x \; [cm]$','Interpreter','latex')
    ylabel('$y \; [cm]$','Interpreter','latex')
    title(name(3),'Interpreter','latex')
    set(gcf, 'Position', [0 0 500 500]);
    ytickformat('%.1f');
    xtickformat('%.1f');
    export_fig(DIR_IMGS + name(1))
    
    mu1 = mean([zero(:,1) - A(pos_zero,2), zero(:,2) - A(pos_zero,3)])
    mu1 = sqrt(mu1(1)^2 + mu1(2)^2)
    covar = det(cov([zero(:,1) - A(pos_zero,2), zero(:,2) - A(pos_zero,3)]))
    %euc = sqrt(mu1 * mu1')
    figure(fig(2))
    histfit(l2norm, 20, 'gamma')
    xlabel('$L2-norm \; [cm]$','Interpreter','latex')
    set(gcf, 'Position', [0 0 500 500]);
    export_fig(DIR_IMGS + name(2))
end