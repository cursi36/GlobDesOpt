function [solver,Robot_infos,dual_arm_copy,JointType_optInfos,LinkLength_optInfos,Distance_optInfo] = readSolverConfig(configFile)

% configFile = "test.xml";
tree = xmlread(configFile);

global robot_idx;
global Robot_infos;
global dual_arm_copy;
global JointType_optInfos;
global LinkLength_optInfos;
global Distance_optInfo;
global solver;

dual_arm_copy = false;
Robot_infos = [];
robot_idx = [];
JointType_optInfos = [];
LinkLength_optInfos = [];
Distance_optInfo = [];
solver = [];

parseChildNodes(tree);

end

% ----- Local function PARSECHILDNODES -----
function [children,SolverOptions] = parseChildNodes(node)
% Recurse over node children.
children = [];
SolverOptions = [];

global robot_idx;
global Robot_infos;
global dual_arm_copy;
global JointType_optInfos;
global LinkLength_optInfos;
global Distance_optInfo;
global solver;

if node.hasChildNodes
    childNodes = node.getChildNodes;
    numChildNodes = childNodes.getLength;
    for count = 1:numChildNodes
        node = childNodes.item(count-1);
        nodeName = char(node.getNodeName);
        bounds = [];
        jnt_limits = [];
        p_bounds = [];
        r_bounds = [];
        switch nodeName
            
            case "optDesConf"
                attributes = parseAttributes(node);
                %                 solver = attributes.Value;
                solver.name = attributes(6).Value;
                solver.acceptRate = str2num(attributes(1).Value);
                solver.MaxIter = str2num(attributes(2).Value);
                solver.Npnts_WS = str2num(attributes(3).Value);
                solver.popSize = str2num(attributes(5).Value);
                solver.NumSeed_BO = str2num(attributes(4).Value);
                
            case "Robot1"
                robot_idx = 1;
                attributes = parseAttributes(node);
                n_joints = str2num(attributes(1).Value);
                origin = str2num(attributes(2).Value);
                rpy = str2num(attributes(3).Value);
                T_init = eye(4,4);
                R = eul2rotm(rpy,'XYZ');
                T_init(1:3,1:3) = R;
                T_init(1:3,4) = origin;
                Robot_infos{robot_idx}.nj = n_joints;
                Robot_infos{robot_idx}.T_init = T_init;
                
            case "dual_arm"
                attributes = parseAttributes(node);
                if attributes(1).Value == "true"
                    dual_arm_copy = true;
                else
                    dual_arm_copy = false;
                end
                idx = str2num(attributes(2).Value);
                Distance_optInfo.idx = idx; %x,y,z
                if ~isempty(idx)
                    bounds(:,1) = str2num(attributes(3).Value);
                    bounds(:,2) = str2num(attributes(4).Value);
                    for n_idx = 1:length(idx)
                        Distance_optInfo.bounds{n_idx} = bounds(n_idx,:); %bounds for x,y,z
                    end
                end
                
            case "Robot2"
                robot_idx = 2;
                attributes = parseAttributes(node);
                n_joints = str2num(attributes(1).Value);
                origin = str2num(attributes(2).Value);
                rpy = str2num(attributes(3).Value);
                R = eul2rotm(rpy,'XYZ');
                T_init = eye(4,4);
                T_init(1:3,1:3) = R;
                T_init(1:3,4) = origin;
                Robot_infos{robot_idx}.nj = n_joints;
                Robot_infos{robot_idx}.T_init = T_init;
                
            case "DH_tab_path"
                attributes = parseAttributes(node);
                DH_tab_file = attributes.Value;
                if ~isempty(DH_tab_file)
                    File = load(DH_tab_file);
                    DH_tab = File.DH_tab;
                else
                    nj = Robot_infos{robot_idx}.nj;
                    DH_tab = zeros(nj,4);
                end
                Robot_infos{robot_idx}.DH_tab = DH_tab;
                
            case "joints"
                attributes = parseAttributes(node);
                type = attributes(2).Value;
                jnt_limits(:,1) = str2num(attributes(1).Value);
                jnt_limits(:,2) = str2num(attributes(3).Value);
                f = find(type == 'r');
                jnt_limits(f,:) = jnt_limits(f,:)*pi/180;
                Robot_infos{robot_idx}.joint_types = type;
                Robot_infos{robot_idx}.joint_limits = jnt_limits;
                
            case "d_idx"
                attributes = parseAttributes(node);
                idx = str2num(attributes(1).Value);
                LinkLength_optInfos{robot_idx}.d_idx = idx;
                if ~isempty(idx)
                    bounds(:,1) = str2num(attributes(2).Value);
                    bounds(:,2) = str2num(attributes(3).Value);
                    for n_idx = 1:length(idx)
                        LinkLength_optInfos{robot_idx}.d_bounds{n_idx} = bounds(n_idx,:);
                    end
                end
                
            case "theta_idx"
                attributes = parseAttributes(node);
                idx = str2num(attributes(1).Value);
                JointType_optInfos{robot_idx}.thetas_idx = idx;
                if ~isempty(idx)
                    bounds(:,1) = str2num(attributes(2).Value);
                    bounds(:,2) = str2num(attributes(3).Value);
                    for n_idx = 1:length(idx)
                        JointType_optInfos{robot_idx}.thetas_bounds{n_idx} = bounds(n_idx,:);
                    end
                end
                
            case "a_idx"
                attributes = parseAttributes(node);
                idx = str2num(attributes(1).Value);
                LinkLength_optInfos{robot_idx}.a_idx = idx;
                if ~isempty(idx)
                    bounds(:,1) = str2num(attributes(2).Value);
                    bounds(:,2) = str2num(attributes(3).Value);
                    for n_idx = 1:length(idx)
                        LinkLength_optInfos{robot_idx}.a_bounds{n_idx} = bounds(n_idx,:);
                    end
                end
                
            case "alpha_idx"
                attributes = parseAttributes(node);
                idx = str2num(attributes(1).Value);
                JointType_optInfos{robot_idx}.alphas_idx = idx;
                if ~isempty(idx)
                    bounds(:,1) = str2num(attributes(2).Value);
                    bounds(:,2) = str2num(attributes(3).Value);
                    for n_idx = 1:length(idx)
                        JointType_optInfos{robot_idx}.alphas_bounds{n_idx} = bounds(n_idx,:);
                    end
                end
                
            case "jnt_idx"
                attributes = parseAttributes(node);
                idx = str2num(attributes(1).Value);
                JointType_optInfos{robot_idx}.idx = idx;
                if ~isempty(idx)
                    p_bounds(:,1) = str2num(attributes(2).Value);
                    p_bounds(:,2) = str2num(attributes(4).Value);
                    r_bounds(:,1) = str2num(attributes(3).Value);
                    r_bounds(:,2) = str2num(attributes(5).Value);
                    for n_idx = 1:length(idx)
                        JointType_optInfos{robot_idx}.bounds{n_idx} = {'p' 'r'};
                        JointType_optInfos{robot_idx}.jointlimits.p{n_idx} = p_bounds(n_idx,:);
                        JointType_optInfos{robot_idx}.jointlimits.r{n_idx} = r_bounds(n_idx,:);
                    end
                end
                
            otherwise
                
        end
        parseChildNodes(node);
        
    end
    %         children(count) = makeStructFromNode(theChild);
end
end

% ----- Local function PARSEATTRIBUTES -----
function attributes = parseAttributes(theNode)
% Create attributes structure.

attributes = [];
if theNode.hasAttributes
    theAttributes = theNode.getAttributes;
    numAttributes = theAttributes.getLength;
    allocCell = cell(1, numAttributes);
    attributes = struct('Name', allocCell, 'Value', ...
        allocCell);
    
    for count = 1:numAttributes
        attrib = theAttributes.item(count-1);
        attributes(count).Name = char(attrib.getName);
        attributes(count).Value = char(attrib.getValue);
    end
end
end
