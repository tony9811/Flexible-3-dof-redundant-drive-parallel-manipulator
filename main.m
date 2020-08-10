clc;clear
tic
p=[[20;0;-500],[20;0;-490],[20;0;-480],[20;0;-470],[0;0;-470],[-20;0;-470],[-20;0;-480],[-20;0;-490],[-20;0;-500]];   
betas = [];

%%%%%%%%%%%%%%%%%%%%%%%%

%Example of inverse solution
for i=1:9

t=inverse(p(:,i),[220;45;140;510]);
betas=[betas,t];%Result of inverse solution

end
hold off

x=p(1,:);
y=p(2,:);
z=p(3,:);
figure(1)%set a work trajectory
plot3(x,y,z);
title('work trajectory') ,xlabel('x-coordinate(mm)'),ylabel('y-coordinate(mm)'),zlabel('z-coordinate（mm）'),axis auto,grid on
betas=betas*pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%

%Example of forward solution
 for i=1:9
  
[x(i),y(i),z(i),k]=forward(betas(1,i),betas(2,i),betas(3,i),betas(4,i),p(:,i));

 end
 pp=[x;y;z];%The end  iteration result
 
%%%%%%%%%%%%%%%%%%%%%%%

 %Draw a Gate track
 p=[-20;0;-500];
 x=p(1);y=p(2);z=p(3);
 for t=0:0.01:sqrt(5.7735*30/50)
      [s,v,a]=guiji(50,t,30); 
      z=[z,-500+s];
 end
 for t=0:0.01:sqrt(5.7735*40/50)
      
            [s,v,a]=guiji(50,t,40);
            x=[x,-20+s];
 end
 for t=0:0.01:sqrt(5.7735*30/50)
       
            [s,v,a]=guiji(50,t,30);
            z=[z,-470-s];
 end
 x1=-20*ones(188);
 x2=20*ones(375-188);
 x=[x1(1,:),x,x2(1,:)];
 y=diag(0*ones(375+216))';
 z=[z(1:188),diag(-470*ones(216))',z(189:375)];
 figure(2)
 comet3(x,y,z)
 title('trajectory planning') ,xlabel('x-coordinate(mm)'),ylabel('y-coordinate（mm）'),zlabel('z-coordinate（mm）'),axis auto,grid on
 
%%%%%%%%%%%%%%%%%%%%%%

% The motion of a quintic polynomial
p=[x;y;z];
betas=[];
for i=1:591

t=inverse(p(:,i),[220;45;140;510]);
betas=[betas,t];%inverse result

end
figure(3)
plot(0.001:0.001:0.591,betas(1,:))
hold on
plot(0.001:0.001:0.591,betas(2,:))
hold on
plot(0.001:0.001:0.591,betas(3,:))
hold on
plot(0.001:0.001:0.591,betas(4,:))
title('angular displacement') ,xlabel('time(s)'),ylabel('angular displacement(rad)'),axis auto,grid on

dbeta1=diff(betas(1,:));
dbeta2=diff(betas(2,:));
dbeta3=diff(betas(3,:));
dbeta4=diff(betas(4,:));
dbetas=[dbeta1;dbeta2;dbeta3;dbeta4];

figure(4)
plot(0.001:0.001:0.590,dbetas(1,:))
hold on
plot(0.001:0.001:0.590,dbetas(2,:))
hold on
plot(0.001:0.001:0.590,dbetas(3,:))
hold on
plot(0.001:0.001:0.590,dbetas(4,:))
title('angular velocity') ,xlabel('time(s)'),ylabel('angular velocity(rad/s)'),axis auto,grid on

ddbeta1=diff(dbeta1);
ddbeta2=diff(dbeta2);
ddbeta3=diff(dbeta3);
ddbeta4=diff(dbeta4);
ddbetas=[ddbeta1;ddbeta2;ddbeta3;ddbeta4];

figure(5)
plot(0.001:0.001:0.589,ddbetas(1,:))
hold on
plot(0.001:0.001:0.589,ddbetas(2,:))
hold on
plot(0.001:0.001:0.589,ddbetas(3,:))
hold on
plot(0.001:0.001:0.589,ddbetas(4,:))
title('angular acceleration') ,xlabel('time(s)'),ylabel('angular acceleration(rad/s^2)'),axis auto,grid on

%%%%%%%%%%%%%%%%%%%%%%
 
%Draw a workspace
tu=[];
for beta1=-pi/3:pi/20:4*pi/9
    for beta2=-pi/3:pi/20:4*pi/9
        for beta3=-pi/3:pi/20:4*pi/9
            for beta4=-pi/3:pi/20:4*pi/9
               
                [xx,yy,zz]=kongjian_zhengjie(beta1,beta2,beta3,beta4);
                tu=[tu;xx,yy,zz];
              
            end
        end
    end
end
figure(6);
pcshow([tu(:,1),tu(:,2),tu(:,3)],[0,0,0],'MarkerSize',5) %Point cloud，[0,0,0]is colour，'MarkerSize',5 is the size of the point
title('workspace') ,xlabel('x-coordinate（mm)'),ylabel('y-coordinate（mm)'),zlabel('z-coordinate（mm)'),axis auto,grid on
figure(7);
plot(tu(:,1),tu(:,3))
title('section of workspace') ,xlabel('x-coordinate（mm)'),ylabel('z-coordinate（mm)'),axis auto,grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%
toc
