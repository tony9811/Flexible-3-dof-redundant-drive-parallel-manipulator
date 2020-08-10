% Inverse kinematics solution
function [betas] = inverse(position,geometry)
p=position;
R=geometry(1); % Stationary platform radius
r=geometry(2);% Moving platform radius
l1=geometry(3);% Master arm length
l2=geometry(4);% Driven arm length
x=p(1);
y=p(2);
z=p(3);
alpha1=0;%The Angle between the master arm and the axis
alpha2=pi/2;
alpha3=pi;
alpha4=3*pi/2;

b1=R*[cos(alpha1);sin(alpha1);0];% Activate joint vector OB
b2=R*[cos(alpha2);sin(alpha2);0];
b3=R*[cos(alpha3);sin(alpha3);0];
b4=R*[cos(alpha4);sin(alpha4);0];

a=r*[cos(alpha1);sin(alpha1);0];% Driven arm vector O'P
a=r*[cos(alpha2);sin(alpha2);0];
a=r*[cos(alpha3);sin(alpha3);0];
a=r*[cos(alpha4);sin(alpha4);0];

J1=2*l1*z;
J2=2*l1*z;
J3=2*l1*z;
J4=2*l1*z;

I1=2*l1*((x+r*cos(alpha1)-R*cos(alpha1))*cos(alpha1)+(y+r*sin(alpha1)-R*sin(alpha1))*sin(alpha1));
I2=2*l1*((x+r*cos(alpha2)-R*cos(alpha2))*cos(alpha2)+(y+r*sin(alpha2)-R*sin(alpha2))*sin(alpha2));
I3=2*l1*((x+r*cos(alpha3)-R*cos(alpha3))*cos(alpha3)+(y+r*sin(alpha3)-R*sin(alpha3))*sin(alpha3));
I4=2*l1*((x+r*cos(alpha4)-R*cos(alpha4))*cos(alpha4)+(y+r*sin(alpha4)-R*sin(alpha4))*sin(alpha4));

K1=(x+r*cos(alpha1)-R*cos(alpha1))^2+(y+r*sin(alpha1)-R*sin(alpha1))^2+z^2+l1^2-l2^2;
K2=(x+r*cos(alpha2)-R*cos(alpha2))^2+(y+r*sin(alpha2)-R*sin(alpha2))^2+z^2+l1^2-l2^2;
K3=(x+r*cos(alpha3)-R*cos(alpha3))^2+(y+r*sin(alpha3)-R*sin(alpha3))^2+z^2+l1^2-l2^2;
K4=(x+r*cos(alpha4)-R*cos(alpha4))^2+(y+r*sin(alpha4)-R*sin(alpha4))^2+z^2+l1^2-l2^2;

if J1^2-K1^2+I1^2<0
return
end

beta1=2*atan((-J1-sqrt(J1^2-K1^2+I1^2))/(K1+I1));
beta2=2*atan((-J2-sqrt(J2^2-K2^2+I2^2))/(K2+I2));
beta3=2*atan((-J3-sqrt(J3^2-K3^2+I3^2))/(K3+I3));
beta4=2*atan((-J4-sqrt(J4^2-K4^2+I4^2))/(K4+I4));



betas=[beta1;beta2;beta3;beta4]*180/pi;
