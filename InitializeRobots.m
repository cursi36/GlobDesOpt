%Function to initlize the robot structures.
%- DH_tabs: cell array of DH tables defined as |d|theta|a|alpha| for each
%robot;
%- joint_types: cell array of joint types defined as 'p' = prismatic; 'r' =
%revolute
%- joint_limits: cell array of joint limis defined as [min max] (in
%radiants)
%- T_inits: cell array of 4x4 base poses
%- dual_arm_copy: if true, generates two arms with same parameters
%- returns: cell array of Robot_calss
function Robots = InitializeRobots(DH_tabs,joint_types,joint_limits,T_inits,dual_arm_copy)

N_robots = length(DH_tabs);
if dual_arm_copy == true
    
    %Take only one and make the other a copy
    DH_tab = DH_tabs{1};
    type = joint_types{1};
    T_init = T_inits{1};
    
    %First robot
    Robots{1} = Robot_class(DH_tab,type, T_init);
    f = find(type == 'r');
    Robots{1}.m_joint_limits = joint_limits{1};
    Robots{1}.m_joint_limits(f,:) = Robots{1}.m_joint_limits(f,:);
    
    %Second robot
    Robots{2} = Robots{1};
    Robots{2}.m_T_init = T_inits{2};
    
else
    
    for i = 1:N_robots
        
        DH_tab = DH_tabs{i};
        type = joint_types{i};
        T_init = T_inits{i};
        f = find(type == 'r');
        
        Robots{i} = Robot_class(DH_tab,type, T_init);
        Robots{i}.m_joint_limits = joint_limits{i};
        Robots{i}.m_joint_limits(f,:) = Robots{i}.m_joint_limits(f,:);
    end
end

% figure()
% sgtitle("Intial robots")
% for i = 1:length(Robots)
%     q = zeros(Robots{i}.m_n_jnts);
%     name = "Robot_"+num2str(i);
%     Robots{i}.Visualize(q,name);
%     hold on
%     
% end

end