function [dtsPs,shps,Vs,reaches,ave_dext] = generateWS_DualDext(Robots,dual_arm_copy,Npnts,acceptRate,plot_en)
disp("Generating WS ")
rng(42);

if dual_arm_copy == true
    N_Robots = 1;
    dist_vect = Robots{2}.m_T_init(1:3,4);
else
    N_Robots = length(Robots);
    if N_Robots > 1
        dist_vect = Robots{2}.m_T_init(1:3,4);
    else
        dist_vect = [1;0;0];
    end
end


for n_rob = 1:N_Robots
    
    nj = Robots{n_rob}.m_n_jnts;
    
    Mr_i = zeros(Npnts,1);
    P_i = zeros(3,Npnts);
    
    n_cart = min(6,nj);
    J_i = zeros(n_cart,nj,Npnts);
    
    iter = 1;
    
    
    lims = Robots{n_rob}.m_joint_limits; %nj x2
    nj = size(lims,1);
    Q_rand = (lims(:,2)-lims(:,1)).*rand(nj,Npnts)+lims(:,1);
    
    for i=1:Npnts
        
        
        qn = Q_rand(:,i);
        
        [Pose,J] = Robots{n_rob}.getFwdKine(qn,"ee");
        
        J_i (:,:,iter) = J(1:n_cart,:);
        
        sigma = svd(J*J');
        d = prod(sigma);
        Mr_i(iter,1) = (d)^(1/size(J,1));
        
        P_i(:,iter)=Pose(1:3,4);
        
        P_proj_i(iter) = P_i(:,iter)'*dist_vect/norm(dist_vect);
        
        iter = iter+1;
        
    end
    
    P{n_rob} = P_i;
    Mr{n_rob} = Mr_i;
    J_iter{n_rob} = J_i;
    
    %     P_base = Robots{n_rob}.m_T_init(1:3,1:3)'*P_i-Robots{n_rob}.m_T_init(1:3,4); %distance from base
    
    if n_rob == 1
        reaches(n_rob) = max(P_proj_i);
    elseif n_rob == 2
        
        P_proj_i = P_proj_i-norm(dist_vect);
        reaches(n_rob) = abs(min(P_proj_i));
    end
end


if dual_arm_copy == true
    P{2} = Robots{2}.m_T_init(1:3,1:3)*P{1}+Robots{2}.m_T_init(1:3,4);
    Mr{2} = Mr{1};
    J_iter{2} = J_iter{1};
    
    reaches(2) = reaches(1);
end

accpt = acceptRate;
%get dexterous points
N_robots = length(Robots);
for i = 1:N_robots
    
    Mr_i = Mr{i};
    mrindex = find( Mr_i >= min(Mr_i)+(max(Mr_i)-min(Mr_i))*accpt);
    Pi = P{i};
    dtsP = Pi(:,mrindex);
    dtsPs{i} = dtsP';
end

if N_robots > 1
%     [shps,Vs,dtsPs{3}] = getIntersectionVolume(dtsPs{1},dtsPs{2});
    [shps,Vs,dtsPs{3},Idx_common] = getIntersectionVolumeIndexes(dtsPs{1},dtsPs{2});
else
    dtsP = dtsPs{1};
    shps{1} = alphaShape(dtsP(:,1),dtsP(:,2),dtsP(:,3));
    Vs(1) = volume(shps{1});
    
end

if N_robots > 1
% compute dueal arm dext only for some random common points
dext_dual = 0;
dext_max = 0;
ave_dext = 0;
if isempty(Idx_common{1}) == 0 && isempty(Idx_common{2}) == 0
    
    N_pnts_dual = min(10^6,0.4*length(Idx_common{1}));
    N_pnts_dual = floor(sqrt(N_pnts_dual));
    
    l1 = randperm(length(Idx_common{1}),N_pnts_dual);
    l2 = randperm(length(Idx_common{2}),N_pnts_dual);
    
    Idx_common{1} = Idx_common{1}(l1);
    Idx_common{2} = Idx_common{2}(l2);
    
    Js_1 = J_iter{1}(:,:,Idx_common{1});
    Js_2 = J_iter{2}(:,:,Idx_common{2});
    
    for i = 1:N_pnts_dual
        
        for j = 1:N_pnts_dual
            [~,dext] = getDualArmDexterity(Js_1(:,:,i),Js_2(:,:,j));
            dext_max = max(dext,dext_max);
        end
        
        dext_dual = dext_dual+dext_max;
    end
    
    ave_dext = dext_dual/size(Js_1,3);
    
end

else
    
   ave_dext = 1;
   
end


% disp("WS Volume "+num2str(V)+" Elapsed "+num2str(t_el))

if plot_en == true
    
    disp("PLOTTING WS")
    
    Colors = [1 0 0;0 0 1;1 1 0];
    fAlphas = [0.3 0.3 0.5];
    
    figure(2)
    clf
    legend_str = [];
    sgtitle("Dext WS point Cloud")
    for i = 1:length(dtsPs)
        hold on
        plot3(dtsPs{i}(:,1),dtsPs{i}(:,2),dtsPs{i}(:,3),'.','Color', Colors(i,:))
        if i < 3
            legend_str = [legend_str;{"Robot "+num2str(i)}];
        end
    end
    xlabel("x")
    ylabel("y")
    zlabel("z")
    legend(legend_str)
    view(3)
    box on
    grid on
    
    figure(3)
    clf
    legend_str = [];
    sgtitle("WS alphashape")
    subplot(1,2,1)
    for i = 1:length(P)
        Pi = P{i}';
        shp_full{i} = alphaShape(Pi(:,1),Pi(:,2),Pi(:,3));
        hold on
        plot(shp_full{i},'EdgeColor','k','EdgeAlpha',0.1,'FaceColor',Colors(i,:),'FaceAlpha',fAlphas(i))
        if i < 3
            legend_str = [legend_str;{"WS Robot "+num2str(i)}];
        end
    end
    xlabel("x")
    ylabel("y")
    zlabel("z")
    legend(legend_str)
    view(3)
    title("Reach WS")
    box on
    grid on
    
    legend_str = [];
    subplot(1,2,2)
    for i = 1:length(shps)
        hold on
        plot(shps{i},'EdgeColor','k','EdgeAlpha',0.1,'FaceColor',Colors(i,:),'FaceAlpha',fAlphas(i))
        if i < 3
            legend_str = [legend_str;{"WS Robot "+num2str(i)}];
        end
    end
    xlabel("x")
    ylabel("y")
    zlabel("z")
    legend(legend_str)
    view(3)
    title("Dext WS")
    box on
    grid on
    
    
    figure(4)
    clf
    legend_str = [];
    sgtitle("Dext WS 3D volume")
    for n_shps = 1:length(shps)
        [bf,vert] = boundaryFacets(shps{n_shps});
        if isempty(bf) == 0
            vert_smooth = SurfaceSmooth(vert,bf,0.1, [], [], [], []);
            hold on
            p = patch('Faces',bf,'Vertices',vert_smooth);
            p.FaceColor = Colors(n_shps,:);
            p.EdgeColor = 'none';
            p.FaceAlpha = fAlphas(n_shps);
            ps{n_shps} = p;
        end
        daspect([1 1 1])
        view(3);
        axis tight
        camlight
        lighting gouraud
        if n_shps < 3
            legend_str = [legend_str;{"WS Robot "+num2str(n_shps)}];
        end
    end
    xlabel("x")
    ylabel("y")
    zlabel("z")
    legend(legend_str)
    view(3)
    box on
    grid on
    
    figure(5)
    clf
    legend_str = [];
    sgtitle("Dext WS +Robots")
    for i = 1:length(Robots)
        q = zeros(Robots{i}.m_n_jnts,1);
        name = "Robot_"+num2str(i);
        Robots{i}.Visualize(q,name);
        hold on
        legend_str = [legend_str;{"Robot "+num2str(i)}];
    end
    for n_shps = 1:length(shps)
        if isempty(shps{n_shps}.Points) == 0
            p = ps{n_shps};
            p = patch('Faces',p.Faces,'Vertices',p.Vertices);
            p.FaceColor = Colors(n_shps,:);
            p.EdgeColor = 'none';
            p.FaceAlpha = fAlphas(n_shps);
        end
        daspect([1 1 1])
        view(3);
        axis tight
        camlight
        lighting gouraud
        if n_shps < 3
            legend_str = [legend_str;{"WS Robot "+num2str(n_shps)}];
        end
    end
    xlabel("x")
    ylabel("y")
    zlabel("z")
    legend(legend_str)
    view(3)
    box on
    grid on
    
    figure(6)
    clf
    legend_str = [];
    sgtitle("Dext and reach WS")
    for n_shps = 1:length(shps)
        if isempty(shps{n_shps}.Points) == 0
            p = ps{n_shps};
            p = patch('Faces',p.Faces,'Vertices',p.Vertices);
            p.FaceColor = Colors(n_shps,:);
            p.EdgeColor = 'none';
            p.FaceAlpha = fAlphas(n_shps);
        end
        daspect([1 1 1])
        view(3);
        axis tight
        camlight
        lighting gouraud
        if n_shps < 3
            legend_str = [legend_str;{"WS Robot "+num2str(n_shps)}];
        end
    end
    xlabel("x")
    ylabel("y")
    zlabel("z")
    legend(legend_str)
    view(3)
    box on
    grid on
    
    % add ful WS plot
    fAlphas_full = [0.1 0.1];
    figure(6)
    hold on
    for n_shps = 1:length(shp_full)
        [bf,vert] = boundaryFacets(shp_full{n_shps});
        if isempty(bf) == 0
            vert_smooth = SurfaceSmooth(vert,bf,0.1, [], [], [], []);
            hold on
            p = patch('Faces',bf,'Vertices',vert_smooth);
            p.FaceColor = Colors(n_shps,:);
            p.EdgeColor = 'none';
            p.FaceAlpha = fAlphas_full(n_shps);
            ps{n_shps} = p;
        end
        daspect([1 1 1])
        view(3);
        axis tight
        camlight
        lighting gouraud
    end
    box on
    grid on
    
end

end