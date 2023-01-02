%% Pose of robot w.r.t world
TR_W = transl(-2,3,5)*trotz(10, "deg")*troty(-5, "deg")*trotx(5, "deg")
trplot(TR_W, 'frame', 'TR_W', "color", "k","axis", [-3,3,0,5,0,8])
hold on 

%% Pose of the Sensor w.r.t robot
TS_R = transl(0,0,1)
% Pose of the Sensor w.r.t world
TS_W = TR_W*TS_R
trplot(TS_W, 'frame', 'TS_W', "color", "r")


%% Pose of the object w.r.t world
TS_O = [0.1730, -0.9811, 0.0872, 2.2486;
    0.9797, 0.1805, 0.0868, -2.8482;
    -0.1009, 0.0704, 0.9924, -4.9789;
    0, 0, 0, 1]
TO_W = TS_W*TS_O
trplot(TO_W, 'frame', 'TO_W', "color", "g")

%% angles
tr2rpy(TO_W,"deg")
%% proof that the Rotations component is valid
det(t2r(TO_W))