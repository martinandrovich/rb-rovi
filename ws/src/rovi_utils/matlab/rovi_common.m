
% PLEASE LOAD THIS USING:
% run("rovi_common.m")

disp("Resolving directories for ROVI project...")

% define directories
DIR_CURRENT    = erase(strrep(which(mfilename),"\","/"), "/" + mfilename + ".m");
DIR_EXPORT_FIG = DIR_CURRENT + "/export_fig";
DIR_ROOT       = DIR_CURRENT + "/../../../..";
DIR_ASSETS     = DIR_ROOT + "/assets";
DIR_WS         = DIR_ROOT + "/ws";
DIR_DATA       = DIR_WS + "/src/rovi_system/data";
DIR_IMGS       = DIR_ASSETS + "/img";

% configure export_fig
% https://github.com/altmany/export_fig
addpath(DIR_EXPORT_FIG);

% configure figure defaults
set(groot, "DefaultFigureRenderer", "painters");
set(groot, "DefaultFigurePosition", [0 0 500 500]);
set(groot, "DefaultFigureColor", [1 1 1]);

% colors
COL_GRAY = [200 200 200]/255;
COL_LIGHTGRAY = [220 220 220]/255;
COL_ORANGE = [255 143 0]/255;
COL_BLUE = [0 207 255]/255;
COL_MAGENTA = [236 88 234]/255;

% log info
disp("DIR_ROOT: " + what(DIR_ROOT).path)
% display("DIR_DATA: " + what(DIR_DATA).path)
disp("DIR_IMGS: " + what(DIR_IMGS).path)