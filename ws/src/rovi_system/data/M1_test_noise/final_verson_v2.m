close all; clf;
run("rovi_common.m");
A = readmatrix('rovi_pose_M1.csv')

A = A * 100

zero = find(A(:,8) == 0)
five = find(A(:,8) == 5*100)
ten = find(A(:,8) == 10*100)
fiften = find(A(:,8) == 15*100)
twenty = find(A(:,8) == 20*100)
twentyfive = find(A(:,8) == 25*100)

plot_experiment_3(80, zero, A, ["std_dev0.pdf", "hist_std0.pdf", "$\mathcal{N}(\mu = 0, \sigma^2 = 0)$"], [1 2])

%% Experiments goes here

plot_experiment_3(80, zero, A, ["std_dev0.pdf", "hist_std0.pdf", "$\mathcal{N}(\mu = 0, \sigma^2 = 0)$"], [1 2])
plot_experiment_3(40, five, A, ["std_dev10.pdf", "hist_std10.pdf", "$\mathcal{N}(\mu = 0, \sigma^2 = 3^2)$"], [3 4])
plot_experiment_3(25, ten, A, ["std_dev20.pdf", "hist_std20.pdf", "$\mathcal{N}(\mu = 0, \sigma^2 = 6^2)$"], [5 6])
plot_experiment_3(18, fiften, A, ["std_dev30.pdf", "hist_std30.pdf","$\mathcal{N}(\mu = 0, \sigma^2 = 9^2)$"], [7 8])
plot_experiment_3(10, twenty, A, ["std_dev40.pdf", "hist_std40.pdf","$\mathcal{N}(\mu = 0, \sigma^2 = 12^2)$"], [9 10])
plot_experiment_3(4, twentyfive, A, ["std_dev50.pdf", "hist_std50.pdf","$\mathcal{N}(\mu = 0, \sigma^2 = 15^2)$"], [11 12])

%% Scatter plot of experiment 1
figure(13)
xlabel('$x [cm]$','Interpreter','latex')
ylabel('$y [cm]$','Interpreter','latex')
scatter(A(zero,5), A(zero,6))

export_fig("hi.pdf")


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

export_fig("plot1.pdf")

%% Matrix plot




