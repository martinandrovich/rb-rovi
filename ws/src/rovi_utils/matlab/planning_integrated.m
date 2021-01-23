close all; clear; clc;
run("rovi_common.m");

EXPERIMENT = "20210122_173731";
DIR_EXPERIMENT = DIR_DATA + "/planning_integrated/" + EXPERIMENT;

%% binary histograms
clc; close all;

fig = figure("Position", [0 0 800 500]);

for i = 0:2
    data = readmatrix(DIR_EXPERIMENT + "/pick_location_" + i + "_attempts.csv");
    attempts   = data(:, 2);
    subplot(1,3,i+1)
    histogram(attempts, "FaceColor", MATLAB_COLORS{i+1})
    xticks([0 1])
    xticklabels(["False", "True"])
%     legend(["Pick location " + (i + 1)])
end

han = axes(fig, "visible", "off"); 
han.XLabel.Visible = "on";
han.YLabel.Visible = "on";
ylabel(han, "Count");
xlabel(han, "Planning attempt success");

export_fig(DIR_IMGS + "/integration/pick-attempts-hist.pdf", "-painters")

%% binomial plots
clc; close all;

figure
hold on

for i = 0:2

    data = readmatrix(DIR_EXPERIMENT + "/pick_location_" + i + "_attempts.csv");
    attempts   = data(:, 2);
    num_trials = length(attempts);
    range      = 0:num_trials;
    num_true   = sum(attempts(:) == 1);
    num_false  = sum(attempts(:) == 0);
    
    pd = fitdist(num_true, "Binomial", "NTrials", num_trials);
    phat = binofit(num_true, num_trials);
    y = binopdf(range, num_trials, phat);
    
    plot(range, y, "LineWidth", 3, "Color", MATLAB_COLORS{i+1})
    xlabel("Planning attempt success")
    ylabel("Probability")
    
end

legend(["Pick location 1", "Pick location 2", "Pick location 3"])

export_fig(DIR_IMGS + "/integration/pick-attempts-binpdf.pdf", "-painters")

%% histograms
clc; close all;
data = readmatrix(DIR_EXPERIMENT + "/pick_location_0_attempts.csv");

attempts   = data(:, 2);
num_trials = length(attempts);
range      = 0:num_trials;
num_true   = sum(attempts(:) == 1);
num_false  = sum(attempts(:) == 0);

pd = fitdist(num_true, "Binomial", "NTrials", num_trials)
phat = binofit(num_true, num_trials)
y = binopdf(range, num_trials, phat);

figure
bar(range, y, 1, "FaceColor", COL_BLUE)
% xlabel("Observation")
xlabel("Observation (planning attempt success)")
ylabel("Probability")

figure
plot(range, y, "LineWidth", 3, "Color", COL_BLUE)
xlabel("Observation (planning attempt success)")
ylabel("Probability")

figure
histogram(attempts, "FaceColor", COL_BLUE)
xticks([0 1])
xticklabels(["False", "True"])
xlabel("Observation (planning attempt success)")
ylabel("Count")