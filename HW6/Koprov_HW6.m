%% Robot and targets Poses
sawyer = SE3;
sawyer.t = [0 0 0.93];
target1 = SE3;
target1.t = [0.75 -0.35 1.014]
target2 = SE3.rpy([0 0 -0.785]);
target2.t = [1 0 1.014]
target3 = SE3.rpy([0 0 0.586931]);
target3.t = [0.85 0.25 1.014]
%% Part 1: A. Poses of the three targets with the repect to the robot base 
% frame
sawyerTtarget1 = inv(sawyer)*target1
sawyerTtarget2 = inv(sawyer)*target2
sawyerTtarget3 = inv(sawyer)*target3

%% Part 1: B. The joint variables to grasp the object at each location, ie.
% inverse kinematics 
L(1) = Link('alpha', -pi/2, 'a', 0.081, 'd', 0);
L(2) = Link('alpha', pi/2, 'a', 0, 'd', 0.191, 'offset', pi/2);
L(3) = Link('alpha', -pi/2, 'a', 0, 'd', 0.399);
L(4) = Link('alpha', pi/2, 'a', 0, 'd', -0.1683);
L(5) = Link('alpha', -pi/2, 'a', 0, 'd', 0.3965);
L(6) = Link('alpha', pi/2, 'a', 0, 'd', 0.136);
L(7) = Link('alpha', 0, 'a', 0, 'd', 0.1785);
R1 = SerialLink(L, 'name', "Sawyer");

pose1 = sawyerTtarget1*SE3.Ry(pi/2)*SE3.Rz(-pi/2);
pose2 = sawyerTtarget2*SE3.Ry(pi/2)*SE3.Rz(-pi/2);
pose3 = sawyerTtarget3*SE3.Ry(pi/2)*SE3.Rz(-pi/2);
pose4 = pose1; % discard position
pose4.t = [0.65 0.4 0.084];

q1dwn = R1.ikine(pose1);
% R1.plot(q1dwn);
q1up = q1dwn -[0 pi/6 0 0 0 0 0];
% R1.teach(q1up)

q2dwn = R1.ikine(pose2);
% R1.teach(q2dwn)
q2up = q2dwn -[0 pi/6 0 0 0 0 0];
% R1.teach(q2up)

q3dwn = R1.ikine(pose3);
% R1.teach(q3dwn)
q3up = q3dwn -[0 pi/6 0 0 0 0 0];
% R1.teach(q3up)

q4dwn = R1.ikine(pose4);
% R1.teach(q4dwn);
q4up = q4dwn -[0 pi/6 0 0 0 0 0];
% R1.teach(q4up)
%%
% * in radians
in_radians = array2table([q1dwn; q2dwn; q3dwn;q4dwn], ...
    'VariableNames', {'q1','q2','q3','q4','q5','q6','q7'}, ...
    'RowNames', {'pos1dwn','pos2dwn','pos3dwn','pos4dwn'} )
%%
% * in degrees
in_degrees = array2table(rad2deg([q1dwn; q2dwn; q3dwn;q4dwn]), ...
    'VariableNames', {'q1','q2','q3','q4','q5','q6','q7'}, ...
    'RowNames', {'pos1dwn','pos2dwn','pos3dwn','pos4dwn'} )
%% Part 1: C.The Jacobian matrix at each location using the joint variables
disp("Jacobian for target1")
R1.jacob0(q1dwn)
disp("Jacobian for target2")
R1.jacob0(q2dwn)
disp("Jacobian for target3")
R1.jacob0(q3dwn)
disp("Jacobian for target4")
R1.jacob0(q4dwn)