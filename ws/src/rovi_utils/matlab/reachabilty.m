close all; clear; clc;
run("rovi_common.m");
%%

data = readmatrix(DIR_DATA + "/reachability/reachability.csv");
% quiver3(zeros(3,1),zeros(3,1),zeros(3,1),[1;0;0],[0;1;0],[0;0;1])

increment = 0.1;
x = [min(data(:, 2)) max(data(:, 2))];
y = [min(data(:, 1)) max(data(:, 1))];

C = [];
for i = 1:size(data)
    row = data(i, 1) * (1/increment) + 1;
    col = data(i, 2) * (1/increment) + 1;
    C(row, col) = data(i, 3);
end

figure()
set(gcf, 'Position', [0 0 500 250]);
imagesc(x, y, C)
colorbar
set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
axis image;
xlabel("Table width (y)")
ylabel("Table height (x)")
ytickformat('%.1f');
xtickformat('%.1f');

export_fig(DIR_IMGS + "/reachability.pdf")