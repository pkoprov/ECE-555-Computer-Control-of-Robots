R = [0 -0.866 0.6;
    0.5 nan 0.75;
    -0.866 0.25 0.433]
%% Pitch
phi = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2))
%% Roll
theta = atan2(R(3,2)/cos(phi),R(3,3)/cos(phi))
%% Yaw
psi = atan2(R(2,1)/cos(phi), R(1,1)/cos(phi))
%% X
x = cos(psi)*cos(theta)+sin(psi)*sin(phi)*sin(theta)
%% checiking the results
R(2,2) = x

tr2rpy(r2t(R), 'deg')