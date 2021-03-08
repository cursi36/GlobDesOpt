% Set the optimization varibales.
% - LinkLength_optInfos is a cell array of structures to set what link lingth
% to optimize for and their bounds, for each robot.
% - JointType_optInfos is a cell array of structures to set what link lingth
% to optimize for and their bounds ['p','r'], for each robot.
% - Distance_optInfos is a structure to chosse if optimize for the distance between the bases of the robots.
% - dual_amr_copy set true otpimizes for same variables in bot arms.
% - returns vector of Bayesopt optimization variables
function [optVars,Indexes] = setOptimizationVariables(LinkLength_optInfos,JointType_optInfos,Distance_optInfo,dual_arm_copy)

distance_name = "dist_";

optVars = [];
Indexes{1}.dual_arm_copy = dual_arm_copy;
if dual_arm_copy == true
    
    Joint_name = "Joint_1_";
    
    %only take info for one arm. The other is just a copy.
    
    % Set d opt variables
    LinkLength_info = LinkLength_optInfos{1};
    Indexes{1}.d = LinkLength_info.d_idx;
    
    LL_name = "d_1_";
    for i = 1:length(LinkLength_info.d_idx)
        
        idx = LinkLength_info.d_idx(i);
        name = LL_name+num2str(idx);
        bounds = LinkLength_info.d_bounds{i};
        
        %         optVar_i = optimizableVariable(char(name),bounds,'Type','integer');
        
        optVar_i.name = char(name);
        optVar_i.bounds = bounds;
        
        optVars = [optVars,{optVar_i}];
        
    end
    
    Indexes{1}.a = LinkLength_info.a_idx;
    % Set a opt variables
    LL_name = "a_1_";
    for i = 1:length(LinkLength_info.a_idx)
        
        idx = LinkLength_info.a_idx(i);
        name = LL_name+num2str(idx);
        bounds = LinkLength_info.a_bounds{i};
        
        %         optVar_i = optimizableVariable(char(name),bounds,'Type','integer');
        
        optVar_i.name = char(name);
        optVar_i.bounds = bounds;
        
        optVars = [optVars,{optVar_i}];
        
    end
    
    %Set joint type opt variables
    JointType_optInfo = JointType_optInfos{1};
    
    %%%thetas
    Indexes{1}.thetas = JointType_optInfo.thetas_idx;
    theta_name = "theta_1_";
    for i = 1:length(JointType_optInfo.thetas_idx)
        
        idx = JointType_optInfo.thetas_idx(i);
        name = theta_name+num2str(idx);
        bounds = JointType_optInfo.thetas_bounds{i};
        
        %         optVar_i = optimizableVariable(char(name),bounds,'Type','integer');
        
        optVar_i.name = char(name);
        optVar_i.bounds = bounds;
        
        optVars = [optVars,{optVar_i}];
        
    end
    
    %%%alphas
    Indexes{1}.alphas = JointType_optInfo.alphas_idx;
    alpha_name = "alpha_1_";
    for i = 1:length(JointType_optInfo.alphas_idx)
        
        idx = JointType_optInfo.alphas_idx(i);
        name = alpha_name+num2str(idx);
        bounds = JointType_optInfo.alphas_bounds{i};
        
        %         optVar_i = optimizableVariable(char(name),bounds,'Type','integer');
        
        optVar_i.name = char(name);
        optVar_i.bounds = bounds;
        
        optVars = [optVars,{optVar_i}];
        
    end
    %joints types
    Indexes{1}.joints = JointType_optInfo.idx;
    
    for i = 1:length(JointType_optInfo.idx)
        
        idx = JointType_optInfo.idx(i);
        name = Joint_name+num2str(idx);
        bounds = JointType_optInfo.bounds{i};
        
            bounds_int = zeros(1,2);
            for ib = 1:2
                
                if bounds{ib} == 'p'
                    bounds_int(ib) = 1;
                elseif bounds{ib} == 'r'
                    bounds_int(ib) = 0;
                end
                
            end
            
            %             optVar_i = optimizableVariable(char(name),bounds,'Type','categorical');
            
            optVar_i.name = char(name);
            optVar_i.bounds = sort(bounds_int);
            
            optVars = [optVars,{optVar_i}];
        
    end
    
    Indexes{1}.jointlimits.p = cell2mat(JointType_optInfo.jointlimits.p);
    Indexes{1}.jointlimits.r = cell2mat(JointType_optInfo.jointlimits.r);
    
    Indexes{1}.jointlimits.p = reshape(Indexes{1}.jointlimits.p,2,length(JointType_optInfo.idx))';
    Indexes{1}.jointlimits.r = reshape(Indexes{1}.jointlimits.r,2,length(JointType_optInfo.idx))';
    
    
else
    
    for n_Robots = 1:length(LinkLength_optInfos)
        
        LL_name = "d_"+num2str(n_Robots)+"_";
        Joint_name = "Joint_"+num2str(n_Robots)+"_";
        
        LinkLength_info = LinkLength_optInfos{n_Robots};
        
        Indexes{n_Robots}.d = LinkLength_info.d_idx;
        for i = 1:length(LinkLength_info.d_idx)
            
            idx = LinkLength_info.d_idx(i);
            name = LL_name+num2str(idx);
            bounds = LinkLength_info.d_bounds{i};
            
            %             optVar_i = optimizableVariable(char(name),bounds,'Type','integer');
            
            optVar_i.name = char(name);
            optVar_i.bounds = bounds;
            
            optVars = [optVars,{optVar_i}];
            
        end
        
        LL_name = "a_"+num2str(n_Robots)+"_";
        Indexes{n_Robots}.a = LinkLength_info.a_idx;
        for i = 1:length(LinkLength_info.a_idx)
            
            idx = LinkLength_info.a_idx(i);
            name = LL_name+num2str(idx);
            bounds = LinkLength_info.a_bounds{i};
            
            %             optVar_i = optimizableVariable(char(name),bounds,'Type','integer');
            
            optVar_i.name = char(name);
            optVar_i.bounds = bounds;
            
            optVars = [optVars,{optVar_i}];
            
        end
        
        %Set joint type opt variables
        JointType_optInfo = JointType_optInfos{n_Robots};
        
        %%%thetas
        Indexes{n_Robots}.thetas = JointType_optInfo.thetas_idx;
        theta_name = "theta_"+num2str(n_Robots)+"_";
        for i = 1:length(JointType_optInfo.thetas_idx)
            
            idx = JointType_optInfo.thetas_idx(i);
            name = theta_name+num2str(idx);
            bounds = JointType_optInfo.thetas_bounds{i};
            
            %         optVar_i = optimizableVariable(char(name),bounds,'Type','integer');
            
            optVar_i.name = char(name);
            optVar_i.bounds = bounds;
            
            optVars = [optVars,{optVar_i}];
            
        end
        
        %%%alphas
        Indexes{n_Robots}.alphas = JointType_optInfo.alphas_idx;
        alpha_name = "alpha_"+num2str(n_Robots)+"_";
        for i = 1:length(JointType_optInfo.alphas_idx)
            
            idx = JointType_optInfo.alphas_idx(i);
            name = alpha_name+num2str(idx);
            bounds = JointType_optInfo.alphas_bounds{i};
            
            %         optVar_i = optimizableVariable(char(name),bounds,'Type','integer');
            
            optVar_i.name = char(name);
            optVar_i.bounds = bounds;
            
            optVars = [optVars,{optVar_i}];
            
        end
        
        %joints types
        Indexes{n_Robots}.joints = JointType_optInfo.idx;
        for i = 1:length(JointType_optInfo.idx)
            
            idx = JointType_optInfo.idx(i);
            name = Joint_name+num2str(idx);
            bounds = JointType_optInfo.bounds{i};
            
            bounds_int = zeros(1,2);
            for ib = 1:2
                
                if bounds{ib} == 'p'
                    bounds_int(ib) = 1;
                elseif bounds{ib} == 'r'
                    bounds_int(ib) = 0;
                end
                
            end
            
            %             optVar_i = optimizableVariable(char(name),bounds,'Type','categorical');
            
            optVar_i.name = char(name);
            optVar_i.bounds = sort(bounds_int);
            
            optVars = [optVars,{optVar_i}];
            
        end
        
        Indexes{n_Robots}.jointlimits.p = cell2mat(JointType_optInfo.jointlimits.p);
        Indexes{n_Robots}.jointlimits.r = cell2mat(JointType_optInfo.jointlimits.r);
        
        Indexes{n_Robots}.jointlimits.p = reshape(Indexes{n_Robots}.jointlimits.p,2,length(JointType_optInfo.idx))';
        Indexes{n_Robots}.jointlimits.r = reshape(Indexes{n_Robots}.jointlimits.r,2,length(JointType_optInfo.idx))';
        
    end
    
    
    
end
%Set distance opt variables
dir = 'xyz';
if ~isempty(Distance_optInfo)
Indexes{2}.dist = Distance_optInfo.idx;
for i = 1:length(Distance_optInfo.idx)
    
    idx = Distance_optInfo.idx(i);
    name = distance_name+num2str(dir(idx));
    bounds = Distance_optInfo.bounds{i};
    
    %     optVar_i = optimizableVariable(char(name),bounds,'Type','integer');
    
    optVar_i.name = char(name);
    optVar_i.bounds = bounds;
    
    optVars = [optVars,{optVar_i}];
    
end
end
end