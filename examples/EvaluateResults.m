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

folder = "Results_Matlab_Dual_BayesOPt_MaxDext/accept_"+ num2str(accept_rate*100);
% folder = "Results_Matlab_Dual_ga_MaxDext/accept_"+ num2str(accept_rate*100);
% folder = "Results_Matlab_Dual_pso_MaxDext/accept_"+ num2str(accept_rate*100);
load(folder+"/Robots");
load(folder+"/elapsed");

[dtsPs,shps,Vs,Safety,ave_dext] = getWSVolumes(Robots,dual_arm_copy,accept_rate,Npnts_WS,cost_fcn,false);

V = ave_dext*Vs(3);
%with safety measure.
SafetyVolume = max(V*Safety,1e-16);
Cost = -log10(SafetyVolume);

Vs = log10(Vs);
ave_dext = log10(ave_dext);

end