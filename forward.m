function[x,y,z,k]=forward(beta1,beta2,beta3,beta4,p)
% forword model
R=220; % Stationary platform radius
r=45;% Moving platform radius
l1=140;% Master arm length
l2=510;% Driven arm length
x=p(1);
y=p(2);
z=p(3);
alpha1=0;% The Angle between the master arm and the axis
alpha2=pi/2;
alpha3=pi;
alpha4=3*pi/2;
f1=((l1*cos(beta1)+R-r)*cos(alpha1)-x)^2+((l1*cos(beta1)+R-r)*sin(alpha1)-y)^2+(l1*sin(beta1)+z)^2-l2^2;
f2=((l1*cos(beta2)+R-r)*cos(alpha2)-x)^2+((l1*cos(beta2)+R-r)*sin(alpha2)-y)^2+(l1*sin(beta2)+z)^2-l2^2;
f3=((l1*cos(beta3)+R-r)*cos(alpha3)-x)^2+((l1*cos(beta3)+R-r)*sin(alpha3)-y)^2+(l1*sin(beta3)+z)^2-l2^2;
f4=((l1*cos(beta4)+R-r)*cos(alpha4)-x)^2+((l1*cos(beta4)+R-r)*sin(alpha4)-y)^2+(l1*sin(beta4)+z)^2-l2^2;
F=[f1;f2;f3;f4];
%% Constraint equation

        df1=[2*x - 2*cos(alpha1)*(R - r + l1*cos(beta1)),2*y - 2*cos(alpha1)*(R - r + l1*cos(beta1)),2*z + 2*l1*sin(beta1)];
        df2=[2*x - 2*cos(alpha2)*(R - r + l1*cos(beta2)),2*y - 2*cos(alpha2)*(R - r + l1*cos(beta2)),2*z + 2*l1*sin(beta2)];
        df3=[2*x - 2*cos(alpha3)*(R - r + l1*cos(beta3)),2*y - 2*cos(alpha3)*(R - r + l1*cos(beta3)),2*z + 2*l1*sin(beta3)];
        df4=[2*x - 2*cos(alpha4)*(R - r + l1*cos(beta4)),2*y - 2*cos(alpha4)*(R - r + l1*cos(beta4)),2*z + 2*l1*sin(beta4)];
        dF=[df1;df2;df3;df4];
%% Take the first partial derivative of the constraint equation

p1=p-dF\F;k=0;%% Get the new position, now go into the loop, and decide

%%Loop and repeat
    while norm(p1-p)>10^-5||k<100
        p=p1;% x=p(1);y=p(2);z=p(3);
        f1=((l1*cos(beta1)+R-r)*cos(alpha1)-x)^2+((l1*cos(beta1)+R-r)*sin(alpha1)-y)^2+(l1*sin(beta1)+z)^2-l2^2;
        f2=((l1*cos(beta2)+R-r)*cos(alpha2)-x)^2+((l1*cos(beta2)+R-r)*sin(alpha2)-y)^2+(l1*sin(beta2)+z)^2-l2^2;
        f3=((l1*cos(beta3)+R-r)*cos(alpha3)-x)^2+((l1*cos(beta3)+R-r)*sin(alpha3)-y)^2+(l1*sin(beta3)+z)^2-l2^2;
        f4=((l1*cos(beta4)+R-r)*cos(alpha4)-x)^2+((l1*cos(beta4)+R-r)*sin(alpha4)-y)^2+(l1*sin(beta4)+z)^2-l2^2;
        F=[f1;f2;f3;f4];
        df1=[2*x - 2*cos(alpha1)*(R - r + l1*cos(beta1)),2*y - 2*cos(alpha1)*(R - r + l1*cos(beta1)),2*z + 2*l1*sin(beta1)];
        df2=[2*x - 2*cos(alpha2)*(R - r + l1*cos(beta2)),2*y - 2*cos(alpha2)*(R - r + l1*cos(beta2)),2*z + 2*l1*sin(beta2)];
        df3=[2*x - 2*cos(alpha3)*(R - r + l1*cos(beta3)),2*y - 2*cos(alpha3)*(R - r + l1*cos(beta3)),2*z + 2*l1*sin(beta3)];
        df4=[2*x - 2*cos(alpha4)*(R - r + l1*cos(beta4)),2*y - 2*cos(alpha4)*(R - r + l1*cos(beta4)),2*z + 2*l1*sin(beta4)];
        dF=[df1;df2;df3;df4];
        p1=p-dF\F;k=k+1;
    end
    p=p1;x=p(1);y=p(2);z=p(3);
