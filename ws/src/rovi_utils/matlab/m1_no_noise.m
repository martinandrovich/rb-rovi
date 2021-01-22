close all; clear; clc;
run("rovi_common.m");

%%

A = readmatrix(DIR_DATA + "/M1_test_noise/rovi_pose_M1.csv");

A = A * 100

zero = find(A(:,8) == 0)
five = find(A(:,8) == 5*100)
ten = find(A(:,8) == 10*100)
fiften = find(A(:,8) == 15*100)
twenty = find(A(:,8) == 20*100)
twentyfive = find(A(:,8) == 25*100)

%% Experiments goes here

[A_zero, mu1, covar1] = plot_experiment_3(40, zero, A, ["std_dev0.pdf", "hist_std0.pdf", "$\mathcal{N}(\mu = 0, \sigma^2 = 0)$"], [1 2])
%%
[A_ten,  mu1, covar1] = plot_experiment_3(32, five, A, ["std_dev10.pdf", "hist_std10.pdf", "$\mathcal{N}(\mu = 0, \sigma^2 = 10^2)$"], [3 4])
%%
[A_twenty,  mu1, covar1] = plot_experiment_3(24, ten, A, ["std_dev20.pdf", "hist_std20.pdf", "$\mathcal{N}(\mu = 0, \sigma^2 = 20^2)$"], [5 6])
%%
[A_thirdteen,  mu1, covar1] = plot_experiment_3(16, fiften, A, ["std_dev30.pdf", "hist_std30.pdf","$\mathcal{N}(\mu = 0, \sigma^2 = 30^2)$"], [7 8])
%%
[A_fourteen,  mu1, covar1] = plot_experiment_3(8, twenty, A, ["std_dev40.pdf", "hist_std40.pdf","$\mathcal{N}(\mu = 0, \sigma^2 = 40^2)$"], [9 10])
%%
[A_fifteen,  mu1, covar1] = plot_experiment_3(4, twentyfive, A, ["std_dev50.pdf", "hist_std50.pdf","$\mathcal{N}(\mu = 0, \sigma^2 = 50^2)$"], [11 12])

%%
clf; close all;
X = [ A_zero ; A_ten ; A_twenty ; A_thirdteen ; A_fourteen ; A_fifteen ]
Y = [ zeros(length(zero),1) ; ones(length(five),1)*10 ; ones(length(ten),1)*20 ; ones(length(fiften),1)*30 ; ones(length(twenty),1)*40; ones(length(twentyfive),1)*50 ]

histogram2(X,Y, [20,20],"FaceAlpha", 1, "FaceColor", "flat", "Normalization", "pdf",'ShowEmptyBins','off')
xlim([0 2.5])
view(75.2, 37.1)
set(gcf, 'Position', [0 0 750 500]);
xlabel('L2-norm [cm]')
ylabel('\sigma')
export_fig(DIR_IMGS + 'hist3dplot.pdf')

%%

length(find(A_zero > 1.1))
length(find(A_ten > 1.1))
length(find(A_twenty > 1.1))
length(find(A_thirdteen > 1.1))
length(find(A_fourteen > 1.1))
length(find(A_fifteen > 1.1))



%% Scatter plot of experiment 1

figure(13)
xlabel('$x [cm]$','Interpreter','latex')
ylabel('$y [cm]$','Interpreter','latex')
scatter(A(zero,5), A(zero,6))


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
xlabel("$\chi_2^2$ quantiles", "Interpreter", "latex")
ylabel("Malanobis quantiles", "Interpreter", "latex")
