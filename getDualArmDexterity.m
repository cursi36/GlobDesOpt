function [H,dext] = getDualArmDexterity(J1,J2)
%ellipse v'*H^-1*v

H1 = J1*J1';
H2 = J2*J2';

[~,S1,V1] = svd(H1); %eigenvalues of H are semiaxes of ellipse squared
% [~,S2,V2] = svd(H2); %eigenvalues of H are semiaxes of ellipse squared

ax_length1 = sqrt(diag(S1));
% ax_length2 = sqrt(diag(S2));

ax_intersect = ax_length1;
for i = 1:size(V1,2)
    v1 = V1(:,i);
    
    %elongation along axis v to reach ellipse 2
    k = pinv(v1'*pinv(H2)*v1);
    
%     if k < 0
%         disp("error in dext")
%     end
k = real(sqrt(k));

if k < ax_length1(i)
  ax_intersect(i) = k;
  
end
end

dext = prod(ax_intersect)^1/2;
H = [];
% S = diag(ax_intersect.^2);
% V = V1;
% H = V*S*V';
end