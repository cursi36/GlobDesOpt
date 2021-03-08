%Cost function to minimize
function [Cost] = getCostFunction(x,Robots,Indexes,acceptRate,Npnts_WS,plot_en)


Robots = generateRobots(x,Robots,Indexes);
dual_arm_copy = Indexes{1}.dual_arm_copy;

[~,~,Vs,Safety] = getWSVolumes(Robots,dual_arm_copy,acceptRate,Npnts_WS,plot_en);

if length(Vs) > 1
    V = Vs(3);
else
    V = Vs(1);
    
end

%with safety measure.
SafetyVolume = max(V*Safety,1e-16);
Cost = -log10(SafetyVolume);

% disp("| Volume | "+num2str(Vs(3))+...
%     " | Saefty | "+num2str(Safety));
end