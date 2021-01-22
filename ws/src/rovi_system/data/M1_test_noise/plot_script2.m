A = readmatrix('rovi_pose_M1.csv')

A = A

zero = find(A(:,8) == 0)
five = find(A(:,8) == 5)
ten = find(A(:,8) == 10*100)
fiften = find(A(:,8) == 15*100)
twenty = find(A(:,8) == 20*100)
twentyfive = find(A(:,8) == 25*100)

figure(1)
subplot(3,1,1)
X = (A(zero,2) - A(zero,5))*100
Y = (A(zero,3) - A(zero,6))*100
Z = sqrt(X.^2 + Y.^2)
h = histogram(Z < 1,'Normalization','probability');
xticklabels({' ','False',' ','True',' '})
ylabel('Normalized probability')
subplot(3,1,2)
histogram(Z, 10)
Z = log(Z)
%qqplot(Z)
%histogram(Z)
R = gamrnd(5,2,1e3,1);
gpdf = fitdist(R,'gamma');
[phat,phatci] = gamfit(R);


[h,p] = chi2gof(Z,'Alpha',0.01)
mu = mean(Z)
sigma_2 = var(Z)
x = -5:0.1:0
y = mvnpdf(x',mu,sigma_2);
subplot(3,1,3)
plot(x,y,'x')
subplot(3,1,3)
y_exp = exp(y)
plot(exp(x),y_exp)
exp(mu)
exp(sigma_2)

