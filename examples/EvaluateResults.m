function EvaluateResults()
clear all
clc
close all
addpath('../SmoothSurf')
addpath('../')
addpath("Data/")

accept_rate = 0.6;
Npnts_WS = 50*1e03;
dual_arm_copy = true;
cost_fcn = "MaxDualDext";
% cost_fcn = "MaxDextVolume";

folder = "Results_Matlab_Dual_BayesOPt/accept_"+ num2str(accept_rate*100);
% folder = "Results_Matlab_Dual_ga/accept_"+ num2str(accept_rate*100);
% folder = "Results_Matlab_Dual_pso/accept_"+ num2str(accept_rate*100);
load(folder+"/Robots");

[dtsPs,shps,Vs,Safety,ave_dext] = getWSVolumes(Robots,dual_arm_copy,accept_rate,Npnts_WS,cost_fcn,true);

end