close all; clear; clc;
run("rovi_common.m");
%%

M1_no_noise = readmatrix(DIR_ASSETS + "/data/M1_test/M1_test/rovi_pose_M1.csv");
data = readmatrix(DIR_ASSETS + "/data/M1_test/M1_no_noise/rovi_pose_M1.csv");
%M1_noise_10 = readmatrix(DIR_ASSETS + "/data/M1_test/M1_test_10/rovi_pose_M1.csv");
%M1_noise_20 = readmatrix(DIR_ASSETS + "/data/M1_test/M1_test_20/rovi_pose_M1.csv");

clf;
j = 0

for i = 1:3
    
    subplot(1, 3, i)
    
    %X = [ M1_no_noise(1+j*50 : 50+j*50, 5) ]
    
    %Y = [ M1_no_noise(1+j*50 : 50+j*50, 6) ]
    
    %Z = [X Y]
    
    %N = hist3(Z)
    
    %N_pcolor = N'
    
    %N_pcolor(size(N_pcolor,1)+1,size(N_pcolor,2)+1) = 0;
    
    %xl = linspace(min(X), max(X), size(N_pcolor,2)); % Columns of N_pcolor
    
    %yl = linspace(min(Y), max(Y), size(N_pcolor,1)); % Rows of N_pcolor
    
    %pcolor(xl, yl, N_pcolor)
    %colormap('hot')
    
    scatter(M1_no_noise(1+j*50, 2),M1_no_noise(1+j*50, 3),'o')
    
    hold on;

    scatter(M1_no_noise(1+j*50 : 50+j*50, 5),M1_no_noise(1+j*50 : 50+j*50,6),'x')
    
    hold on;
    
    scatter(data(1+j*20:20+j*20,5),data(1+j*20:20+j*20,6),'^')
    
    legend('original','icp','no icp');

    %scatter(M1_noise_10(1+j*50 : 50+j*50, 5),M1_noise_10(1+j*50 : 50+j*50,6),'x')
    
    %scatter(M1_noise_20(1+j*50 : 50+j*50, 5),M1_noise_20(1+j*50 : 50+j*50,6),'*')
    
    pbaspect([1 1 1])
    
    xlim( [ M1_no_noise(1+j*50, 2)-0.02 , M1_no_noise( 1 + j*50, 2 )+0.02 ] )
    ylim( [ M1_no_noise(1+j*50, 3)-0.02 , M1_no_noise( 1 + j*50, 3 )+0.02 ] )
    
    ytickformat('%.3f');
    xtickformat('%.3f');
    
    j = j + 1
    
end

%j = 0
%hold on;
%for i = 1:3
%    subplot(1, 4, i)
    %hold on;
    %scatter(data(1+j*20,2),data(1+j*20,3),'*')
    %h%old on
    %scatter(data(1+j*20:20+j*20,5),data(1+j*20:20+j*20,6),'^')
    %hold on;
    %scatter(data_noise(1+j*20:20+j*20,5),data_noise(1+j*20:20+j*20,6),'x')
    %pbaspect([1 1 1])
    %ytickformat('%.3f');
    %xtickformat('%.3f');
    %xlabel("x [m]")
    %ylabel("y [m]")
    %j = j + 1
%end

%subplot(1,4,4)
%hold on;
%plot(0,0,  0,0,  0,0,  0,0)
%axis off
%legend('original','icp','no icp','Location','NorthEastOutside');

figure
set(gcf, 'Position', [0 0 1500 1500]);
export_fig(DIR_IMGS + "/variance_dense_stereo.pdf")