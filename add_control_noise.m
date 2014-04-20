function [V,W]= add_control_noise(V,W,Q, addnoise)
%
% Add random noise to nominal control values. We assume Q is diagonal.

if addnoise == 1
    V= V + randn(1)*sqrt(Q(1,1));
    W(1)= W(1) + randn(1)*sqrt(Q(2,2));
    W(2)= W(2);
end

