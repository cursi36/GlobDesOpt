function Robots = generateRobots(optVars,Robots,Indexes)

dual_arm_copy = Indexes{1}.dual_arm_copy;

if dual_arm_copy == true
    
    N_robots = 1;
    
else
    
    N_robots = length(Robots);
end

optVars_cell = optVars;
robot_i_numOpts = 0;

for i = 1:N_robots
    
    d_idx = Indexes{i}.d;
    theta_idx = Indexes{i}.thetas;
    a_idx = Indexes{i}.a;
    alpha_idx = Indexes{i}.alphas;
    joints_idx = Indexes{i}.joints;
    p_jointlimits = Indexes{i}.jointlimits.p;
    r_jointlimits = Indexes{i}.jointlimits.r;
    
    n_ds = length(d_idx);
    n_thetas = length(theta_idx);
    n_as = length(a_idx);
    n_alphas = length(alpha_idx);
    n_js = length(joints_idx);
    
    opt_ds = optVars_cell(robot_i_numOpts+1:robot_i_numOpts+n_ds);
    robot_i_numOpts = robot_i_numOpts+n_ds;
    
    opt_as = optVars_cell(robot_i_numOpts+1:robot_i_numOpts+n_as);
    robot_i_numOpts = robot_i_numOpts+n_as;
    
    opt_thetas = optVars_cell(robot_i_numOpts+1:robot_i_numOpts+n_thetas)*pi/180;
    robot_i_numOpts = robot_i_numOpts+n_thetas;
    
    opt_alphas = optVars_cell(robot_i_numOpts+1:robot_i_numOpts+n_alphas)*pi/180;
    robot_i_numOpts = robot_i_numOpts+n_alphas;
    
    opt_joints = optVars_cell(robot_i_numOpts+1:robot_i_numOpts+n_js);
    robot_i_numOpts = robot_i_numOpts+n_js;
    
    %update ds
    Robots{i}.m_DH_table(d_idx,1) = opt_ds;
    
    %update as
    Robots{i}.m_DH_table(a_idx,3) = opt_as;
    
        %update thetass
    Robots{i}.m_DH_table(theta_idx,2) = opt_thetas;
    
    %update alphas
    Robots{i}.m_DH_table(alpha_idx,4) = opt_alphas;
    
    %update joint types and its limits
    for jt = 1:length(opt_joints)
        jt_idx = joints_idx(jt);
        type = opt_joints(jt);        
        if type == 1
             Robots{i}.m_type(jt_idx) = 'p';
            
        Robots{i}.m_joint_limits(jt_idx,:) = p_jointlimits(jt,:);
            
        else
            Robots{i}.m_type(jt_idx) = 'r';
            Robots{i}.m_joint_limits(jt_idx,:) = r_jointlimits(jt,:)*pi/180;
        end
        
    end
    
end

if dual_arm_copy == true
    T_init =  Robots{2}.m_T_init;
    Robots{2} = Robots{1};
    Robots{2}.m_T_init = T_init;
end

%adapt initial pose of second robot
if length(Robots) > 1
dist_idx = Indexes{2}.dist;
opt_dist = optVars_cell(robot_i_numOpts+1:end);
Robots{2}.m_T_init(dist_idx,4) = opt_dist;
end
end