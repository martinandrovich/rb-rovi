close all; clear; clc;
run("rovi_common.m");

%%

A = readmatrix(DIR_DATA + "/M1_test_noise/rovi_pose_M1.csv");

A = A * 100

zero = find(A(:,8)       == 0)
five = find(A(:,8)       == 5*100)
ten = find(A(:,8)        == 10*100)
fiften = find(A(:,8)     == 15*100)
twenty = find(A(:,8)     == 20*100)
twentyfive = find(A(:,8) == 25*100)

%% Experiments goes here

[A_zero, mu1, covar1] = plot_experiment_1(40, zero, A,          ["std_dev0.pdf", "hist_std0.pdf", "$\mathcal{N}(\mu = 0, \sigma^2 = 0)$"], [1 2]);
[A_ten,  mu1, covar1] = plot_experiment_1(32, five, A,          ["std_dev3.pdf", "hist_std3.pdf", "$\mathcal{N}(\mu = 0, \sigma^2 = 3^2)$"], [3 4]);
[A_twenty,  mu1, covar1] = plot_experiment_1(24, ten, A,        ["std_dev6.pdf", "hist_std6.pdf", "$\mathcal{N}(\mu = 0, \sigma^2 = 6^2)$"], [5 6]);
[A_thirdteen,  mu1, covar1] = plot_experiment_1(16, fiften, A,  ["std_dev9.pdf", "hist_std9.pdf","$\mathcal{N}(\mu = 0, \sigma^2 = 9^2)$"], [7 8]);
[A_fourteen,  mu1, covar1] = plot_experiment_1(8, twenty, A,    ["std_dev12.pdf", "hist_std12.pdf","$\mathcal{N}(\mu = 0, \sigma^2 = 12^2)$"], [9 10]);
[A_fifteen,  mu1, covar1] = plot_experiment_1(4, twentyfive, A, ["std_dev15.pdf", "hist_std15.pdf","$\mathcal{N}(\mu = 0, \sigma^2 = 15^2)$"], [11 12]);

%%
clf; close all;
X = [ A_zero ; A_ten ; A_twenty ; A_thirdteen ; A_fourteen ; A_fifteen ]
Y = [ zeros(length(zero),1) ; ones(length(five),1)*3 ; ones(length(ten),1)*6 ; ones(length(fiften),1)*9 ; ones(length(twenty),1)*12; ones(length(twentyfive),1)*15 ]

histogram2(X,Y, [20,20],"FaceAlpha", 1, "FaceColor", "flat", "Normalization", "pdf",'ShowEmptyBins','off')
xlim([0 2.5])
view(75.2, 37.1)
set(gcf, 'Position', [0 0 750 500]);
xlabel('L2-norm [cm]')
ylabel('\sigma')
export_fig(DIR_IMGS + '/M1/evaluation/' + 'hist3dplot.pdf', -'painters')

%% Scatter plot of experiment 1

figure(13)
plotmatrix([A(zero,5)-A(zero,2) A(zero,6)-A(zero,3)])
xlabel('x [cm]')
ylabel('y [cm]')
export_fig(DIR_IMGS + '/M1/evaluation/' + 'matrixplot.pdf', -'painters')

%% QQ-plot of experiment 1

figure(14)
x = [A(zero,5)-A(zero,2) A(zero,6)-A(zero,3)]
mu = mean(x)
covar = cov(x)
d = zeros(length(x),1)
for i = 1:length(x)
   d(i) = sqrt((x(i,:) - mu) * inv(covar) * ( x(i,:) - mu)')
end
df = 2
z_i = chi2rnd(df,1,10000);
qqplot(d,z_i)
xlabel("\chi_2^2 quantiles")
ylabel("Malanobis quantiles")
export_fig(DIR_IMGS + '/M1/evaluation/' + 'qqplot.pdf', -'painters')

%% 

figure(15)
x = [A(zero,5)-A(zero,2) A(zero,6)-A(zero,3)]
d = sqrt(x(:,1).^2 + x(:,2).^2)
histfit(d, 50, 'gamma')
xlabel('Eucliean distance [cm]')
export_fig(DIR_IMGS + '/M1/evaluation/' + 'gammaplot.pdf', -'painters')


%% Functions

function [l2norm, mu1, covar] = plot_experiment_1(noise, zero, A, name, fig)
    run("rovi_common.m");
    figure(fig(1))
    pos_zero = zero
    zero = [awgn(A(zero,5),noise,0) awgn(A(zero,6),noise,0)]
    l2norm = sqrt((zero(:,1) - A(pos_zero,2)).^2 + (zero(:,2) - A(pos_zero,3)).^2)
    scatter(zero(:,1), zero(:,2))
    xlabel('x [cm]')
    ylabel('y [cm]')
    set(gcf, 'Position', [0 0 500 500]);
    ytickformat('%.1f');
    xtickformat('%.1f');
    export_fig(DIR_IMGS + '/M1/results/' + name(1), '-painters')
    
    mu1 = mean([zero(:,1) - A(pos_zero,2), zero(:,2) - A(pos_zero,3)])
    mu1 = sqrt(mu1(1)^2 + mu1(2)^2)
    covar = det(cov([zero(:,1) - A(pos_zero,2), zero(:,2) - A(pos_zero,3)]))
    %euc = sqrt(mu1 * mu1')
    figure(fig(2))
    histfit(l2norm, 20, 'gamma')
    xlabel('L2-norm [cm]','Interpreter','latex')
    set(gcf, 'Position', [0 0 500 500]);
    export_fig(DIR_IMGS + '/M1/results/' + name(2),'-painters')
    disp(DIR_IMGS)
end
