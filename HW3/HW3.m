%% final pose of the end-effector
T0 = SE3;
T0.t = [7;3;2];
T1 = trotz(90, "deg")*transl(7,3,2)*trotx(-90, "deg")*transl(4,-3,7);
sprintf("Final pose is")
disp(T1)
%% plotting
figure;
trplot(T0, 'frame', 'T0', 'color', 'k',"axis", [-11,8,2,12,1,6])
hold on;
trplot(T1, 'frame', 'T1', 'color', 'r')
tranimate(T0, T1)
hold off;
%% cubic coefficients calculation
T = [1 0 0 0;
    0 1 0 0;
    1 5 25 125;
    0 1 10 75];
q = [0 0 1 0]';
a = inv(T)*q;
for i = 1:4
    sprintf("a(%1.d)= %2.3f", i,a(i))
end
%% position plotting
tf = 5;
t = [0:0.1:tf];
s = a(1)+a(2)*t+a(3)*power(t,2)+a(4)*power(t,3);
figure;
plot(t,s);
X0 = 7;
Y0 = 3;
Z0 = 2;
Xf = T1(1,4);
Yf = T1(2,4);
Zf = T1(3,4);
X = (1-s)*X0 + s*Xf;
Y = (1-s)*Y0 + s*Yf;
Z = (1-s)*Z0 + s*Zf;
figure;
plot(t,X);
hold on;
plot(t,Y);
plot(t,Z);
legend('X','Y','Z');
%% velocity plotting
ds = a(2)+2*a(3)*t+3*a(4)*power(t,2);
figure;
hold off;
plot(t,ds);
vX = (1-ds)*0 + ds*abs(Xf-X0);
vY = (1-ds)*0 + ds*abs(Yf-Y0);
vZ = (1-ds)*0 + ds*abs(Zf-Z0);
figure;
plot(t,vX);
hold on;
plot(t,vY);
plot(t,vZ);
legend('VX','VY','VZ');
%% angles
svmax = 0.5
q0 = T0.UnitQuaternion;
q1 = SE3(T1).UnitQuaternion;
theta = acos(q0.s*q1.s+sum(q0.v.*q1.v));
qvmax = (sin((1-svmax)*theta)*q0+sin(svmax*theta)*q1)/sin(theta);
Xvmax = (1-svmax)*X0 + svmax*Xf;
Yvmax = (1-svmax)*Y0 + svmax*Yf;
Zvmax = (1-svmax)*Z0 + svmax*Zf;
sprintf("Position at max velocity is [%1.2f,%2.2f,%3.2f]",Xvmax,Yvmax,Zvmax)
sprintf("Orientation at max velocity is %s",qvmax.char)