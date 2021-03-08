%shps = {shp1,shp2,shp_intersect}
%Vs = {V1,V2,V_intersect}
%P in R^mx3
function [shps,Vs,P_common] = getIntersectionVolume(P1,P2)

shps{1} = alphaShape(P1(:,1),P1(:,2),P1(:,3));
Vs(1) = volume(shps{1});

shps{2} = alphaShape(P2(:,1),P2(:,2),P2(:,3));
Vs(2) = volume(shps{2});

%Intersection points
inShapeIdx = inShape(shps{1},P2);
P_commonIdx = find(inShapeIdx == 1);
P_common = P2(P_commonIdx,:);

shps{3} = alphaShape(P_common(:,1),P_common(:,2),P_common(:,3));
Vs(3) = volume(shps{3});
end