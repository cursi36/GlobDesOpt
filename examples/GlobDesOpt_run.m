% Example to optimize for design of two dual arms with different
% structures
function GlobDesOpt_run
clear all;
close all;
clc;

warning('off','all')

addpath('../SmoothSurf')
addpath('../')
addpath("Data/")

%% Initializations
% configFile = "Data/optConf_dualArm.xml"; %DualArm opt
% configFile = "Data/optConf_singleArm.xml"; %SingleArm opt
% configFile = "Data/optConf_dualArm_copy.xml"; %DualArm copy opt
% configFile = "Data/optConf_dualArm_test.xml"; %DualArm copy opt
configFile = "Data/optConf_DualArm_Copy_Run.xml"; %DualArm copy opt
[solver,Robot_infos,dual_arm_copy,JointType_optInfos,LinkLength_optInfos,Distance_optInfo] = readSolverConfig(configFile);

parallel = false;

for i = 1:length(Robot_infos)
    DH_tabs{i} = Robot_infos{i}.DH_tab;
    joint_types{i} = Robot_infos{i}.joint_types;
    joint_limits{i} = Robot_infos{i}.joint_limits;
    T_inits{i} = Robot_infos{i}.T_init;
    
    if dual_arm_copy == true
        DH_tabs{i+1} = DH_tabs{i};
        joint_types{i+1} = Robot_infos{i}.joint_types;
        joint_limits{i+1} = Robot_infos{i}.joint_limits;
        T_inits{i+1} = Robot_infos{i+1}.T_init;
        break;
    end
    
end

Robots = InitializeRobots(DH_tabs,joint_types,joint_limits,T_inits,dual_arm_copy);

if solver.name == "BayesOpt"
    
    [optVars,Indexes] = setOptimizationVariablesBO(LinkLength_optInfos,JointType_optInfos,Distance_optInfo,dual_arm_copy);
    
    
else
    [optVars,Indexes] = setOptimizationVariables(LinkLength_optInfos,JointType_optInfos,Distance_optInfo,dual_arm_copy);
    
end

%% START OPTIMIZATION
Accept_rate = solver.acceptRate;
MaxIter = solver.MaxIter;
Npnts_WS = solver.Npnts_WS; %number of points for WS computation
popSize = solver.popSize;
NumSeed_BO = solver.NumSeed_BO;

disp("******SOLVER Info: "+solver.name+"; MaxIter: "+num2str(MaxIter)+"; N points WS: "+num2str(Npnts_WS)+"; Accept Rate: "+num2str(Accept_rate))
if solver.name == "BayesOpt"
    disp("SeedPoints :"+num2str(NumSeed_BO))
else
    disp("Pop size: "+num2str(popSize))
    
end
for i = 1:length(Accept_rate)
    close all;
    
    accept_rate = Accept_rate(i);
    disp("*******RUNNING")
    
    %%minimizes function
    nvars = length(optVars);
    
    %%%%%GENETIC ALGORITHM%%%%%%%%%%%%%%%%%%
    if solver.name == "ga"
        A = [];
        b = [];
        Aeq = [];
        beq = [];
        nonlcon = [];
        IntCon = [1:nvars]';
        lb = zeros(nvars,1);
        ub = zeros(nvars,1);
        for nv = 1:nvars
            lb(nv) = optVars{nv}.bounds(1);
            ub(nv) = optVars{nv}.bounds(2);
        end
        n_pop = popSize;
        
        if parallel
            parpool;
        end
        options = optimoptions('ga','Display', 'iter','UseVectorized',true,'PopulationSize',n_pop,...
            'UseParallel', true,'PlotFcn','gaplotbestf','CrossOverFcn',{'crossoverheuristic',1.2});
        options.InitialPopulationMatrix = []; %%rows = up to pop size; cols = numb of variables
        options.MaxGenerations = MaxIter;
        options.MaxStallGenerations = 10;
        options.EliteCount = floor(0.2*n_pop);
        
        [x,fval,exitflag,Res,population,scores] = ga(@(x)costFunction(x,solver.name,Robots,Indexes,accept_rate,Npnts_WS,false),...
            nvars,A,b,Aeq,beq,lb,ub,nonlcon,IntCon,options);
        
        if parallel
            delete(gcp);
        end
        
        %%%%%%%%%%%PSO ALGORITHM%%%%%%%%%%%%
    elseif solver.name =="pso"
        lb = zeros(nvars,1);
        ub = zeros(nvars,1);
        for nv = 1:nvars
            lb(nv) = optVars{nv}.bounds(1);
            ub(nv) = optVars{nv}.bounds(2);
        end
        
        SwarmSize = popSize;
        
        if parallel
            parpool;
        end
        
        options = optimoptions('particleswarm','SwarmSize',SwarmSize,...
            'Display', 'iter','MaxIterations',MaxIter,'MaxStallIterations',10, 'PlotFcn','pswplotbestf',...
            'UseParallel',true,'UseVectorized', true);
        
        [x,fval,exitflag,Res] = particleswarm(@(x)costFunction(x,solver.name,Robots,Indexes,accept_rate,Npnts_WS,false),...
            nvars,lb,ub,options);
        if parallel
            delete(gcp);
        end
        
        %%%%%BAYESIAN OPTIMIZATION%%%%%%%%%%%%%
    elseif solver.name == "BayesOpt"
        Res = bayesopt(@(x)costFunction(x,solver.name,Robots,Indexes,accept_rate,Npnts_WS,false),...
            optVars,'Verbose',0,...
            'IsObjectiveDeterministic',true,'AcquisitionFunctionName','expected-improvement-per-second-plus',...
            'ExplorationRatio',0.6,...
            'MaxObjectiveEvaluations',MaxIter,'NumSeedPoints',NumSeed_BO,'UseParallel',parallel);
        x_res = Res.XAtMinObjective;
        x = x_res{:,:};
        
    end
    
    %plot results:
    Robots = generateRobots(x,Robots,Indexes);
    dual_arm_copy = Indexes{1}.dual_arm_copy;
    
    [dtsPs,Vs,SafetyMeasure] = getWSVolumes(Robots,dual_arm_copy,accept_rate,Npnts_WS,true);
    
    folder_base = "Results_Matlab_1/";
    folder = folder_base+"accept_"+num2str(accept_rate*100)+"/";
    mkdir (folder);
    
    
    saveas(figure(2),folder+"V_cloud.fig")
    saveas(figure(3),folder+"V_shp.fig")
    saveas(figure(4),folder+"V_patch.fig")
    save(folder+"ResOpt","Res")
    save(folder+"ParamsOpt","x")
end

end

function [Cost] = costFunction(x,solver,Robots,Indexes,acceptRate,Npnts_WS,plot_en)
if solver == "BayesOpt"
    x = x{:,:};
    Cost = getCostFunction(x,Robots,Indexes,acceptRate,Npnts_WS,plot_en);
else
    x = round(x);
    
    N_pop = size(x,1);
    Cost = zeros(N_pop,1);
    
    Robots_init = Robots;
    
    parfor n_pop = 1:N_pop
        xi = x(n_pop,:);
        Cost(n_pop) = getCostFunction(xi,Robots_init,Indexes,acceptRate,Npnts_WS,plot_en);
    end
end


end

% %Cost function to minimize
% function Cost = getCostFunction(x,Robots,Indexes,acceptRate,Npnts_WS,plot_en)
%
% N_pop = size(x,1);
% Cost = zeros(N_pop,1);
%
% for n_pop = 1:N_pop
%     xi = x(n_pop,:);
%     Robots = generateRobots(xi,Robots,Indexes);
%     dual_arm_copy = Indexes{1}.dual_arm_copy;
%
%     [~,~,Vs,Safety] = getWSVolumes(Robots,dual_arm_copy,acceptRate,Npnts_WS,plot_en);
%
%     if length(Vs) > 1
%         V = Vs(3);
%     else
%         V = Vs(1);
%
%     end
%
%     %with safety measure.
%     SafetyVolume = max(V*Safety,1e-16);
%     Cost(n_pop) = -log10(SafetyVolume);
%
%     % disp("| Volume | "+num2str(Vs(3))+...
%     %     " | Saefty | "+num2str(Safety));
% end
% end



function x = roundValues(x, Resolution)

% unit_scale = 1;
% for i = 1:4
%
%     div_rem = rem(x{1,i}/unit_scale*2,2);
%
%     if div_rem <= 1
%
%         if div_rem >= 0.5
%             x{1,i} = ceil(x{1,i}/unit_scale * 2) / 2;
%         else
%             x{1,i} = floor(x{1,i}/unit_scale * 2) / 2;
%         end
%     end
%
%     if div_rem > 1
%         if  div_rem >= 1.5
%             %round to x+0.5
%             x{1,i} = ceil(x{1,i}/unit_scale * 2) / 2;
%
%         elseif div_rem < 1.5
%             %round to x-0.5
%             x{1,i} = floor(x{1,i}/unit_scale * 2) / 2;
%
%         end
%     end
%
%     x{1,i} = x{1,i}*unit_scale;
% end
end




