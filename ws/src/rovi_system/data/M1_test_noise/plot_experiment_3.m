function [ zero ] = plot_experiment_3(noise, zero, A, name, fig)
    run("rovi_common.m");
    figure(fig(1))
    pos_zero = zero
    zero = [awgn(A(zero,5),noise,0) awgn(A(zero,6),noise,0)]
    lnorm = sqrt((zero(:,1) - A(pos_zero,2)).^2 + (zero(:,2) - A(pos_zero,3)).^2)
    scatter(zero(:,1), zero(:,2))
    xlabel('x [cm]')
    ylabel('y [cm]')
    set(gcf, 'Position', [0 0 500 500]);
    ytickformat('%.2f');
    xtickformat('%.2f');
    export_fig(DIR_CURRENT + name(1))
    
    set(gcf, 'Position', [0 0 500 500]);
    plotmatrix([zero(:,1)-A(pos_zero,2) zero(:,2)-A(pos_zero,3)])
    xlabel('x [cm]')
    ylabel('y [cm]')
    ytickformat('%.2f');
    xtickformat('%.2f');
    export_fig("matrixplot.pdf")
    
    mu1 = mean(zero/100)
    covar = cov(zero/100)
    euc = sqrt(mu1 * mu1')
    figure(fig(2))
    histfit(lnorm, 20, 'gamma')
    xlabel('L2-norm [cm]')
    set(gcf, 'Position', [0 0 500 500]);
    export_fig(DIR_CURRENT + name(2))
end