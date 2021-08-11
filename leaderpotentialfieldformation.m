function leaderpotentialfieldformation
%work area
wa=[0:0.1:13;0:0.1:13];
%locations of obstacles and target
T=[10,10];
obs=[3,4;8,5];
n=length(wa(1,:));
%gains
kT=3;
ko=[4,5];

leadergain=1;
G=leadergain*[1 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0];

Gp=1; Gf=1;%*[1 1 1 1 1 1]; %potential and formation gains
options=odeset('events',@StopSim);
Vp=4; kn=3; L=2; %km=0.05;
% A1=[0 0 0 0 0 0;1 0 0 0 0 0;1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 1 0 0 0];
% D1=[0 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1];
A1=[0 1 1 1 1 1;1 0 1 0 0 1;1 1 0 1 0 0;1 0 1 0 1 0;1 0 0 1 0 1;1 1 0 0 1 0];
D1=[5 0 0 0 0 0;0 3 0 0 0 0;0 0 3 0 0 0;0 0 0 3 0 0;0 0 0 0 3 0;0 0 0 0 0 3];
L1=D1-A1;

A=[0 0 0.5 0 0 0;0.5 0 0 0 0 0.5;0.5 0.5 0 0 0 0;0 0.5 0 0 0 0;0 0 0.5 0 0 0;0 0 0 0.5 0.5 0];
D=[0.5 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 0.5 0 0;0 0 0 0 0.5 0;0 0 0 0 0 1];
% L=D-A;
%initial robot condition
init(1:6)=[0.75;0;0.5;0.5;0.1;0.25];
init(7:12)=[0.75;0.5;0;0.5;0.1;0.25];
init(13:18)=[pi/3;pi/6;pi/3;pi/4;pi/3;pi/4];
init(19:21)=[1.5;2;pi/3];
%0.75;pi/3;0.5;pi/3;0;pi/3;0.5;pi/4;0.1;pi/3;0.25;pi/4
[t,pos]=ode45(@robot,[0 100],init,options);
figure()
hold on
plot(pos(:,1),pos(:,7),pos(:,2),pos(:,8),pos(:,3),pos(:,9),pos(:,4),pos(:,10),pos(:,5),pos(:,11),pos(:,6),pos(:,12),pos(:,19),pos(:,20),'*')
plot(obs(1,1),obs(1,2),'o',obs(2,1),obs(2,2),'o','MarkerFaceColor','r','MarkerSize',10)
title('Testing leader node with position control formation')
%plot(obs(1,1),obs(1,2),'o',obs(2,1),obs(2,2),'o','MarkerFaceColor','r','MarkerSize',10)
plot(T(1,1),T(1,2),'s','MarkerFaceColor','g','MarkerSize',15)
ylabel('y axis')
xlabel('x axis')
legend({'x1','x2','x3','x4','x5','x6','Leader Node','obstacle 1','obstacle 2','target'})
xlim([0 13]);
ylim([0 13]);


    function dp=robot(t,pos)
        dp=zeros(21,1);        
        x(1:6)=pos(1:6);
        y(1:6)=pos(7:12);
        theta(1:6)=pos(13:18);
        xL=pos(19);yL=pos(20);thetaL=pos(21);
        if ((x(1)-T(1))^2)+((y(1)-T(2))^2)>1 || ((x(2)-T(1))^2)+((y(2)-T(2))^2)>1 || ((x(3)-T(1))^2)+((y(3)-T(2))^2)>1 || ((x(4)-T(1))^2)+((y(4)-T(2))^2)>1 || ((x(5)-T(1))^2)+((y(5)-T(2))^2)>1 || ((x(6)-T(1))^2)+((y(6)-T(2))^2)>1
            km=0.05;
        else
            km=0;
        end
        
        
        FLx=kT*(T(1,1)-xL)/(sqrt(((T(1,1)-xL)^2)+((T(1,2)-yL)^2)));
        Fo1Lx=-ko(1,1)*(obs(1,1)-xL)/((((obs(1,1)-xL)^2)+(obs(1,2)-yL)^2)^1.5);
        Fo2Lx=-ko(1,2)*(obs(2,1)-xL)/((((obs(2,1)-xL)^2)+(obs(2,2)-yL)^2)^1.5);
        FLx=FLx+Fo1Lx+Fo2Lx;
        FLy=kT*(T(1,2)-yL)/(sqrt(((T(1,1)-xL)^2)+(T(1,2)-yL)^2));
        Fo1Ly=-ko(1,1)*(obs(1,2)-yL)/((((obs(1,1)-xL)^2)+(obs(1,2)-yL)^2)^1.5);
        Fo2Ly=-ko(1,2)*(obs(2,2)-yL)/((((obs(2,1)-xL)^2)+(obs(2,2)-yL)^2)^1.5);
        thetadesL=(Gp*atan2(FLy,FLx));
        fiL=kn*(thetadesL-thetaL);
                
        
        FTx=kT*(T(1,1)-x(1))/(sqrt(((T(1,1)-x(1))^2)+((T(1,2)-y(1))^2)));
        Fo1x=-ko(1,1)*(obs(1,1)-x(1))/((((obs(1,1)-x(1))^2)+(obs(1,2)-y(1))^2)^1.5);
        Fo2x=-ko(1,2)*(obs(2,1)-x(1))/((((obs(2,1)-x(1))^2)+(obs(2,2)-y(1))^2)^1.5);
        Fom2x=-km*(x(2)-x(1))/((((x(2)-x(1))^2)+(y(2)-y(1))^2)^1.5);
        Fom3x=-km*(x(3)-x(1))/((((x(3)-x(1))^2)+(y(3)-y(1))^2)^1.5);
        Fom4x=-km*(x(4)-x(1))/((((x(4)-x(1))^2)+(y(4)-y(1))^2)^1.5);
        Fom5x=-km*(x(5)-x(1))/((((x(5)-x(1))^2)+(y(5)-y(1))^2)^1.5);
        Fom6x=-km*(x(6)-x(1))/((((x(6)-x(1))^2)+(y(6)-y(1))^2)^1.5);
        Fx=FTx+Fo1x+Fo2x+Fom2x+Fom3x+Fom4x+Fom5x+Fom6x;
        FTy=kT*(T(1,2)-y(1))/(sqrt(((T(1,1)-x(1))^2)+(T(1,2)-y(1))^2));
        Fo1y=-ko(1,1)*(obs(1,2)-y(1))/((((obs(1,1)-x(1))^2)+(obs(1,2)-y(1))^2)^1.5);
        Fo2y=-ko(1,2)*(obs(2,2)-y(1))/((((obs(2,1)-x(1))^2)+(obs(2,2)-y(1))^2)^1.5);
        Fom2y=-km*(y(2)-y(1))/((((x(2)-x(1))^2)+(y(2)-y(1))^2)^1.5);
        Fom3y=-km*(y(3)-y(1))/((((x(3)-x(1))^2)+(y(3)-y(1))^2)^1.5);
        Fom4y=-km*(y(4)-y(1))/((((x(4)-x(1))^2)+(y(4)-y(1))^2)^1.5);        
        Fom5y=-km*(y(5)-y(1))/((((x(5)-x(1))^2)+(y(5)-y(1))^2)^1.5);       
        Fom6y=-km*(y(6)-y(1))/((((x(6)-x(1))^2)+(y(6)-y(1))^2)^1.5);       
        Fy=FTy+Fo1y+Fo2y+Fom2y+Fom3y+Fom4y+Fom5y+Fom6y;
        thetades(1)=(Gp*atan2(Fy,Fx));
%         fi(1)=kn*(thetades(1)-theta);
        
        FTx2=kT*(T(1,1)-x(2))/(sqrt(((T(1,1)-x(2))^2)+((T(1,2)-y(2))^2)));
        Fo1x2=-ko(1,1)*(obs(1,1)-x(2))/((((obs(1,1)-x(2))^2)+(obs(1,2)-y(2))^2)^1.5);
        Fo2x2=-ko(1,2)*(obs(2,1)-x(2))/((((obs(2,1)-x(2))^2)+(obs(2,2)-y(2))^2)^1.5);
        Fom1x2=-km*(x(1)-x(2))/((((x(1)-x(2))^2)+(y(1)-y(2))^2)^1.5);
        Fom3x2=-km*(x(3)-x(2))/((((x(3)-x(2))^2)+(y(3)-y(2))^2)^1.5);
        Fom4x2=-km*(x(4)-x(2))/((((x(4)-x(2))^2)+(y(4)-y(2))^2)^1.5);
        Fom5x2=-km*(x(5)-x(2))/((((x(5)-x(2))^2)+(y(5)-y(2))^2)^1.5);
        Fom6x2=-km*(x(6)-x(2))/((((x(6)-x(2))^2)+(y(6)-y(2))^2)^1.5);
        Fx2=FTx2+Fo1x2+Fo2x2+Fom1x2+Fom3x2+Fom4x2+Fom5x2+Fom6x2;
        FTy2=kT*(T(1,2)-y(2))/(sqrt(((T(1,1)-x(2))^2)+(T(1,2)-y(2))^2));
        Fo1y2=-ko(1,1)*(obs(1,2)-y(2))/((((obs(1,1)-x(2))^2)+(obs(1,2)-y(2))^2)^1.5);
        Fo2y2=-ko(1,2)*(obs(2,2)-y(2))/((((obs(2,1)-x(2))^2)+(obs(2,2)-y(2))^2)^1.5);
        Fom1y2=-km*(y(1)-y(2))/((((x(1)-x(2))^2)+(y(1)-y(2))^2)^1.5);
        Fom3y2=-km*(y(3)-y(2))/((((x(3)-x(2))^2)+(y(3)-y(2))^2)^1.5);
        Fom4y2=-km*(y(4)-y(2))/((((x(4)-x(2))^2)+(y(4)-y(2))^2)^1.5);
        Fom5y2=-km*(y(5)-y(2))/((((x(5)-x(2))^2)+(y(5)-y(2))^2)^1.5);
        Fom6y2=-km*(y(6)-y(2))/((((x(6)-x(2))^2)+(y(6)-y(2))^2)^1.5);
        Fy2=FTy2+Fo1y2+Fo2y2+Fom1y2+Fom3y2+Fom4y2+Fom5y2+Fom6y2;
        thetades(2)=(Gp*atan2(Fy2,Fx2));
%        fi(2)=kn*(thetades(2)-theta2);
        
        FTx3=kT*(T(1,1)-x(3))/(sqrt(((T(1,1)-x(3))^2)+((T(1,2)-y(3))^2)));
        Fo1x3=-ko(1,1)*(obs(1,1)-x(3))/((((obs(1,1)-x(3))^2)+(obs(1,2)-y(3))^2)^1.5);
        Fo2x3=-ko(1,2)*(obs(2,1)-x(3))/((((obs(2,1)-x(3))^2)+(obs(2,2)-y(3))^2)^1.5);
        Fom1x3=-km*(x(1)-x(3))/((((x(1)-x(3))^2)+(y(1)-y(3))^2)^1.5);
        Fom2x3=-km*(x(2)-x(3))/((((x(2)-x(3))^2)+(y(2)-y(3))^2)^1.5);
        Fom4x3=-km*(x(4)-x(3))/((((x(4)-x(3))^2)+(y(4)-y(3))^2)^1.5);
        Fom5x3=-km*(x(5)-x(3))/((((x(5)-x(3))^2)+(y(5)-y(3))^2)^1.5);        
        Fom6x3=-km*(x(6)-x(3))/((((x(6)-x(3))^2)+(y(6)-y(3))^2)^1.5);
        Fx3=FTx3+Fo1x3+Fo2x3+Fom1x3+Fom2x3+Fom4x3+Fom5x3+Fom6x3;
        FTy3=kT*(T(1,2)-y(3))/(sqrt(((T(1,1)-x(3))^2)+(T(1,2)-y(3))^2));
        Fo1y3=-ko(1,1)*(obs(1,2)-y(3))/((((obs(1,1)-x(3))^2)+(obs(1,2)-y(3))^2)^1.5);
        Fo2y3=-ko(1,2)*(obs(2,2)-y(3))/((((obs(2,1)-x(3))^2)+(obs(2,2)-y(3))^2)^1.5);
        Fom1y3=-km*(y(1)-y(3))/((((x(1)-x(3))^2)+(y(1)-y(3))^2)^1.5);
        Fom2y3=-km*(y(2)-y(3))/((((x(2)-x(3))^2)+(y(2)-y(3))^2)^1.5);
        Fom4y3=-km*(y(4)-y(3))/((((x(4)-x(3))^2)+(y(4)-y(3))^2)^1.5);        
        Fom5y3=-km*(y(5)-y(3))/((((x(5)-x(3))^2)+(y(5)-y(3))^2)^1.5); 
        Fom6y3=-km*(y(6)-y(3))/((((x(6)-x(3))^2)+(y(6)-y(3))^2)^1.5);                
        Fy3=FTy3+Fo1y3+Fo2y3+Fom1y3+Fom2y3+Fom4y3+Fom5y3+Fom6y3;
        thetades(3)=(Gp*atan2(Fy3,Fx));
%        fi(3)=kn*(thetades(3)-theta3);

        FTx4=kT*(T(1,1)-x(4))/(sqrt(((T(1,1)-x(4))^2)+((T(1,2)-y(4))^2)));
        Fo1x4=-ko(1,1)*(obs(1,1)-x(4))/((((obs(1,1)-x(4))^2)+(obs(1,2)-y(4))^2)^1.5);
        Fo2x4=-ko(1,2)*(obs(2,1)-x(4))/((((obs(2,1)-x(4))^2)+(obs(2,2)-y(4))^2)^1.5);
        Fom1x4=-km*(x(1)-x(4))/((((x(1)-x(4))^2)+(y(1)-y(4))^2)^1.5);
        Fom2x4=-km*(x(2)-x(4))/((((x(2)-x(4))^2)+(y(2)-y(4))^2)^1.5);
        Fom3x4=-km*(x(3)-x(4))/((((x(3)-x(4))^2)+(y(3)-y(4))^2)^1.5);
        Fom5x4=-km*(x(5)-x(4))/((((x(5)-x(4))^2)+(y(5)-y(4))^2)^1.5);        
        Fom6x4=-km*(x(6)-x(4))/((((x(6)-x(4))^2)+(y(6)-y(4))^2)^1.5);        
        Fx4=FTx4+Fo1x4+Fo2x4+Fom1x4+Fom2x4+Fom3x4+Fom5x4+Fom6x4;
        FTy4=kT*(T(1,2)-y(4))/(sqrt(((T(1,1)-x(4))^2)+(T(1,2)-y(4))^2));
        Fo1y4=-ko(1,1)*(obs(1,2)-y(4))/((((obs(1,1)-x(4))^2)+(obs(1,2)-y(4))^2)^1.5);
        Fo2y4=-ko(1,2)*(obs(2,2)-y(4))/((((obs(2,1)-x(4))^2)+(obs(2,2)-y(4))^2)^1.5);
        Fom1y4=-km*(y(1)-y(4))/((((x(1)-x(4))^2)+(y(1)-y(4))^2)^1.5);
        Fom2y4=-km*(y(2)-y(4))/((((x(2)-x(4))^2)+(y(2)-y(4))^2)^1.5);
        Fom3y4=-km*(y(3)-y(4))/((((x(3)-x(4))^2)+(y(3)-y(4))^2)^1.5);        
        Fom5y4=-km*(y(5)-y(4))/((((x(5)-x(4))^2)+(y(5)-y(4))^2)^1.5);        
        Fom6y4=-km*(y(6)-y(4))/((((x(6)-x(4))^2)+(y(6)-y(4))^2)^1.5);                
        Fy4=FTy4+Fo1y4+Fo2y4+Fom1y4+Fom2y4+Fom3y4+Fom5y4+Fom6y4;
        thetades(4)=(Gp*atan2(Fy4,Fx4));
%        fi(4)=kn*(thetades(4)-theta4);

        FTx5=kT*(T(1,1)-x(5))/(sqrt(((T(1,1)-x(5))^2)+((T(1,2)-y(5))^2)));
        Fo1x5=-ko(1,1)*(obs(1,1)-x(5))/((((obs(1,1)-x(5))^2)+(obs(1,2)-y(5))^2)^1.5);
        Fo2x5=-ko(1,2)*(obs(2,1)-x(5))/((((obs(2,1)-x(5))^2)+(obs(2,2)-y(5))^2)^1.5);
        Fom1x5=-km*(x(1)-x(5))/((((x(1)-x(5))^2)+(y(1)-y(5))^2)^1.5);
        Fom2x5=-km*(x(2)-x(5))/((((x(2)-x(5))^2)+(y(2)-y(5))^2)^1.5);
        Fom3x5=-km*(x(3)-x(5))/((((x(3)-x(5))^2)+(y(3)-y(5))^2)^1.5);
        Fom4x5=-km*(x(4)-x(5))/((((x(4)-x(5))^2)+(y(4)-y(5))^2)^1.5);        
        Fom6x5=-km*(x(6)-x(5))/((((x(6)-x(5))^2)+(y(6)-y(5))^2)^1.5);        
        Fx5=FTx5+Fo1x5+Fo2x5+Fom1x5+Fom2x5+Fom3x5+Fom4x5+Fom6x5;
        FTy5=kT*(T(1,2)-y(5))/(sqrt(((T(1,1)-x(5))^2)+(T(1,2)-y(5))^2));
        Fo1y5=-ko(1,1)*(obs(1,2)-y(5))/((((obs(1,1)-x(5))^2)+(obs(1,2)-y(5))^2)^1.5);
        Fo2y5=-ko(1,2)*(obs(2,2)-y(5))/((((obs(2,1)-x(5))^2)+(obs(2,2)-y(5))^2)^1.5);
        Fom1y5=-km*(y(1)-y(5))/((((x(1)-x(5))^2)+(y(1)-y(5))^2)^1.5);
        Fom2y5=-km*(y(2)-y(5))/((((x(2)-x(5))^2)+(y(2)-y(5))^2)^1.5);
        Fom3y5=-km*(y(3)-y(5))/((((x(3)-x(5))^2)+(y(3)-y(5))^2)^1.5);        
        Fom4y5=-km*(y(4)-y(5))/((((x(4)-x(5))^2)+(y(4)-y(5))^2)^1.5);        
        Fom6y5=-km*(y(6)-y(5))/((((x(6)-x(5))^2)+(y(6)-y(5))^2)^1.5);                
        Fy5=FTy5+Fo1y5+Fo2y5+Fom1y5+Fom2y5+Fom3y5+Fom4y5+Fom6y5;
        thetades(5)=(Gp*atan2(Fy5,Fx5));
%        fi(5)=kn*(thetades(5)-theta5);
        
        FTx6=kT*(T(1,1)-x(6))/(sqrt(((T(1,1)-x(6))^2)+((T(1,2)-y(6))^2)));
        Fo1x6=-ko(1,1)*(obs(1,1)-x(6))/((((obs(1,1)-x(6))^2)+(obs(1,2)-y(6))^2)^1.5);
        Fo2x6=-ko(1,2)*(obs(2,1)-x(6))/((((obs(2,1)-x(6))^2)+(obs(2,2)-y(6))^2)^1.5);
        Fom1x6=-km*(x(1)-x(6))/((((x(1)-x(6))^2)+(y(1)-y(6))^2)^1.5);
        Fom2x6=-km*(x(2)-x(6))/((((x(2)-x(6))^2)+(y(2)-y(6))^2)^1.5);
        Fom3x6=-km*(x(3)-x(6))/((((x(3)-x(6))^2)+(y(3)-y(6))^2)^1.5);
        Fom4x6=-km*(x(4)-x(6))/((((x(4)-x(6))^2)+(y(4)-y(6))^2)^1.5);        
        Fom5x6=-km*(x(5)-x(6))/((((x(5)-x(6))^2)+(y(5)-y(6))^2)^1.5);        
        Fx6=FTx6+Fo1x6+Fo2x6+Fom1x6+Fom2x6+Fom3x6+Fom4x6+Fom5x6;
        FTy6=kT*(T(1,2)-y(6))/(sqrt(((T(1,1)-x(6))^2)+(T(1,2)-y(6))^2));
        Fo1y6=-ko(1,1)*(obs(1,2)-y(6))/((((obs(1,1)-x(6))^2)+(obs(1,2)-y(6))^2)^1.5);
        Fo2y6=-ko(1,2)*(obs(2,2)-y(6))/((((obs(2,1)-x(6))^2)+(obs(2,2)-y(6))^2)^1.5);
        Fom1y6=-km*(y(1)-y(6))/((((x(1)-x(6))^2)+(y(1)-y(6))^2)^1.5);
        Fom2y6=-km*(y(2)-y(6))/((((x(2)-x(6))^2)+(y(2)-y(6))^2)^1.5);
        Fom3y6=-km*(y(3)-y(6))/((((x(3)-x(6))^2)+(y(3)-y(6))^2)^1.5);        
        Fom4y6=-km*(y(4)-y(6))/((((x(4)-x(6))^2)+(y(4)-y(6))^2)^1.5);        
        Fom5y6=-km*(y(5)-y(6))/((((x(5)-x(6))^2)+(y(5)-y(6))^2)^1.5);                
        Fy6=FTy6+Fo1y6+Fo2y6+Fom1y6+Fom2y6+Fom3y6+Fom4y6+Fom5y6;
        thetades(6)=(Gp*atan2(Fy6,Fx6));
%        fi(6)=kn*(thetades(6)-theta6);
        
        nthetades(1:6)=thetades(1:6);%-Gf*L1*transpose(theta(1:6));
        
        MB=L1*(x(1:6)');
        %fink
        fi(1:6)=kn*(thetades(1:6)-theta(1:6));
        dp(1:6)=Vp*cos(fi(1:6)).*cos(theta(1:6))-(Gf*(L1+G)*x(1:6)')'+((L1+G)*xL*ones(6,1))';
        dp(7:12)=Vp*cos(fi(1:6)).*sin(theta(1:6))-(Gf*(L1+G)*y(1:6)')'+((L1+G)*yL*ones(6,1))';
        dp(13:18)=(Vp/L)*sin(fi(1:6));
        dp(19)=Vp*cos(fiL)*cos(thetaL);
        dp(20)=Vp*cos(fiL)*sin(thetaL);
        dp(21)=(Vp/L)*sin(fiL);
%         for im=1:6
%             dp(im)=Vp*cos(fi(im))*cos(theta(im));
%             dp(im+6)=Vp*cos(fi(im))*sin(theta(im));
%             dp(im+12)=(Vp/L)*sin(fi(im));
%         end
%         dp(1)=Vp*cos(fi(1))*cos(theta(1));
%         dp(7)=Vp*cos(fi(1))*sin(theta(1));
%         dp(13)=(Vp/L)*sin(fi(1));
%         dp(2)=Vp*cos(fi(2))*cos(theta(2));
%         dp(8)=Vp*cos(fi(2))*sin(theta(2));
%         dp(14)=(Vp/L)*sin(fi(2));
%         dp(3)=Vp*cos(fi(3))*cos(theta(3));
%         dp(9)=Vp*cos(fi(3))*sin(theta(3));
%         dp(15)=(Vp/L)*sin(fi(3));
%         dp(4)=Vp*cos(fi(4))*cos(theta(4));
%         dp(10)=Vp*cos(fi(4))*sin(theta(4));
%         dp(16)=(Vp/L)*sin(fi(4));
%         dp(5)=Vp*cos(fi(5))*cos(theta(5));
%         dp(11)=Vp*cos(fi(5))*sin(theta(5));
%         dp(17)=(Vp/L)*sin(fi(5));
%         dp(6)=Vp*cos(fi(6))*cos(theta(6));
%         dp(12)=Vp*cos(fi(6))*sin(theta(6));
%         dp(18)=(Vp/L)*sin(fi(6));
    end
    function [Val,Ister,Dir]=StopSim(t,pos)
        if ((pos(19)-T(1))^2)+((pos(20)-T(2))^2)<1e-2
            Val(1)=0;
        elseif ((pos(1)-T(1))^2)+((pos(7)-T(2))^2)<1e-2
            Val(1)=0;
        elseif ((pos(2)-T(1))^2)+((pos(8)-T(2))^2)<1e-2
            Val(1)=0;
        elseif ((pos(3)-T(1))^2)+((pos(9)-T(2))^2)<1e-2
            Val(1)=0;
        elseif ((pos(4)-T(1))^2)+((pos(10)-T(2))^2)<1e-2
            Val(1)=0;
        elseif ((pos(5)-T(1))^2)+((pos(11)-T(2))^2)<1e-2
            Val(1)=0;
        elseif ((pos(6)-T(1))^2)+((pos(12)-T(2))^2)<1e-2
            Val(1)=0;
        else
            Val(1)=1;
        end
        Ister(1)=1;
        Dir(1)=0;
    end 
end