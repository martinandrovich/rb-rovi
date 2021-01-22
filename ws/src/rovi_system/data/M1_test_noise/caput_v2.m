A = readmatrix('rovi_pose_M1.csv')
A = A*100
zero = finddistance(A(:,8) == 0)
five = find(A(:,8) == 5*100)
ten = find(A(:,8) == 10*100)
fiften = find(A(:,8) == 15*100)
twenty = find(A(:,8) == 20*100)
twentyfive = find(A(:,8) == 25*100)

%%
X = (A(zero,2) - A(zero,5))
Y = (A(zero,3) - A(zero,6))
x = [X Y]
mu1 = mean(x)
covar = cov(x)
euc = sqrt(mu1 * mu1')
histfit(sqrt(X.^2 + Y.^2),20,'gamma')

%%
X = (A(five,2) - awgn(A(five,5),25,0))
Y = (A(five,3) - awgn(A(five,6),25,0))
x = [X Y]
mu2 = mean(x)
covar = cov(x)
euc = sqrt(mu2 * mu2')
det(covar)
histfit(sqrt(X.^2 + Y.^2),20,'gamma')
%%
X = (A(ten,2) - awgn(A(ten,5),20,0))
Y = (A(ten,3) - awgn(A(ten,6),20,0))
x = [X Y]
mu2 = mean(x)
covar = cov(x)
euc = sqrt(mu2 * mu2')
det(covar)
histfit(sqrt(X.^2 + Y.^2),20,'gamma')
%%
X = (A(fiften,2) - awgn(A(fiften,5),15,0))
Y = (A(fiften,3) - awgn(A(fiften,6),15,0))
x = [X Y]
mu2 = mean(x)
covar = cov(x)
euc = sqrt(mu2 * mu2')
det(covar)
histfit(sqrt(X.^2 + Y.^2),20,'gamma')
%%
X = (A(twenty,2) - awgn(A(twenty,5),10,0))
Y = (A(twenty,3) - awgn(A(twenty,6),10,0))
x = [X Y]
mu2 = mean(x)
covar = cov(x)
euc = sqrt(mu2 * mu2')
det(covar)
histfit(sqrt(X.^2 + Y.^2),20,'gamma')
%%
X = (A(twentyfive,2) - awgn(A(twentyfive,5),5,0))
Y = (A(twentyfive,3) - awgn(A(twentyfive,6),5,0))
x = [X Y]
mu2 = mean(x)
covar = cov(x)
euc = sqrt(mu2 * mu2')
det(covar)
histfit(sqrt(X.^2 + Y.^2),20,'gamma')
%%
