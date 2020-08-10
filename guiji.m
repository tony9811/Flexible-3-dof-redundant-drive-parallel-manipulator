function [s,v,a]=guiji(am,t,juli)

tt=sqrt(5.7735*juli/am);

tao=t/tt;

a=am*(60*tao-180*tao^2+120*tao^3)/5.7735;
v=am*tt*(30*tao^2-60*tao^3+30*tao^4)/5.7735;
s=am*tt^2*(10*tao^3-15*tao^4+6*tao^5)/5.7735;
 
end
