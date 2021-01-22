A = readmatrix('rovi_pose_M1.csv')

A = A * 100

zero = find(A(:,8) == 0)
five = find(A(:,8) == 5)
ten = find(A(:,8) == 10)
fiften = find(A(:,8) == 15)
twenty = find(A(:,8) == 20)
twentyfive = find(A(:,8) == 25)

%%

figure(1)
pos_zero = zero
zero = [awgn(A(zero,5),80,0) awgn(A(zero,6),80,0)]
lnorm = sqrt((zero(:,1) - A(pos_zero,2)).^2 + (zero(:,2) - A(pos_zero,3)).^2)
scatter(zero(:,1), zero(:,2))
xlabel('$x [cm]$','Interpreter','latex')
ylabel('$y [cm]$','Interpreter','latex')
title('$\mathcal{N}(\mu = 0, \sigma = 0)$','Interpreter','latex')
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperPosition', [0 0 7.5 7.5])
saveas(gcf,'scatter_std_dev0.png')
mu1 = mean(zero/100)
covar = cov(zero/100)
euc = sqrt(mu1 * mu1')

figure(2)
histfit(lnorm, 20, 'gamma')
xlabel('L2-norm [cm]')
saveas(gcf,'hist_std_dev0.png')

%%

figure(2)
five = [awgn(A(five,5),40,0) awgn(A(five,6),40,0)]
scatter(five(:,1), five(:,2))
xlabel('x [cm]')
ylabel('y [cm]')
title('$\mathcal{N}(\mu = 0, \sigma^2 = 10^2)$','Interpreter','latex')
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperPosition', [0 0 7.5 7.5])
saveas(gcf,'std_dev5.png')

mu1 = mean(zero/100)
covar = cov(zero/100)
euc = sqrt(mu1 * mu1')

figure(2)
histfit(lnorm, 20, 'gamma')
xlabel('L2-norm [cm]')
saveas(gcf,'hist_std_dev0.png')

figure(3)
ten = [awgn(A(ten,5),25,0) awgn(A(ten,6),25,0)]
scatter(ten(:,1), ten(:,2))
xlabel('x [cm]')
ylabel('y [cm]')
title('$\mathcal{N}(\mu = 0, \sigma^2 = 20^2)$','Interpreter','latex')
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperPosition', [0 0 7.5 7.5])
saveas(gcf,'std_dev10.png')

figure(4)
fiften = [awgn(A(fiften,5),18,0) awgn(A(fiften,6),18,0)]
scatter(fiften(:,1), fiften(:,2))
xlabel('x [cm]')
ylabel('y [cm]')
title('$\mathcal{N}(\mu = 0, \sigma^2 = 30^2)$','Interpreter','latex')
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperPosition', [0 0 7.5 7.5])
saveas(gcf,'std_dev15.png')

figure(5)
twenty = [awgn(A(twenty,5),14,0) awgn(A(twenty,6),14,0)]
scatter(twenty(:,1), twenty(:,2))
xlabel('x [cm]')
ylabel('y [cm]')
title('$\mathcal{N}(\mu = 0, \sigma^2 = 40^2)$','Interpreter','latex')
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperPosition', [0 0 7.5 7.5])
saveas(gcf,'std_dev20.png')

figure(6)
twentyfive = [awgn(A(twentyfive,5),12,0) awgn(A(twentyfive,6),12,0)]
scatter(twentyfive(:,1), twentyfive(:,2))
xlabel('x [cm]')
ylabel('y [cm]')
title('$\mathcal{N}(\mu = 0, \sigma^2 = 50^2)$','Interpreter','latex')
set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperPosition', [0 0 7.5 7.5])
saveas(gcf,'std_dev25.png')

%%
close all;