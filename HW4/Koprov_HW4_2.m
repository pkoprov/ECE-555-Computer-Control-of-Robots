L(1) = Link([0 1 0 pi/2]);
L(2) = Link([0 1 0 -pi/2]);
L(3) = Link('prismatic', 'theta', 0, 'a', 0, 'alpha', pi/2,'offset', 1);
L(3).qlim = [0 1]
L(4) = Link([0 0 0 -pi/2]);
L(5) = Link([0 1 0 0]);
L(5).qlim = [0 0];


R1 = SerialLink(L, 'name', 'Koprov_HW4');
R1.plotopt = {'workspace' [-2,2,-2,2,-2.5,5]}

R1.teach

%% Position with all theta angles = 0
R1.fkine([0 0 0 0 0])
