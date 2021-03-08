function [dtsPs,shps,Vs,Safety] = getWSVolumes(Robots,dual_arm_copy,acceptRate,Npnts_WS,plot_en)


[dtsPs,shps,Vs,reaches] = generateWS(Robots,dual_arm_copy,Npnts_WS,acceptRate,plot_en);

Safety = 1;
if length(Robots) > 1
L_tot = sum(reaches);

dist = norm(Robots{2}.m_T_init(1:3,4));
% ProbCollision = getProbCollision(dist,L_tot);
Safety = getSafety(dist,L_tot);
end

end

function Pen = getSafety(dist,L_tot)
Pen = 1-exp(-7*dist^2/(L_tot)^2);

end
