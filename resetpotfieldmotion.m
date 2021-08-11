function resetpotfieldmotion
%work area
wa=[0:0.1:25;0:0.1:25];
%locations of obstacles and target
T=[22,22];
obs=[13,14;18,15];
n=length(wa(1,:));
%gains
kT=3;
ko=[4,5];

Gp=1;Gf=[1 1 1 1 1 1]; %potential and formation gains
options=odeset('events',@StopSim);
Vp=4; kn=3; L=2;
A1=[0 0 0 0 0 0;1 0 0 0 0 0;1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 1 0 0 0];
D1=[0 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1];
L1=D1-A1;
init=[0.75;0;0.5;0.75;0.5;0;pi/3;pi/3;pi/4]; %initial robot condition
[t,pos]=ode45(@robot,[0 100],init,options);
figure()
plot(pos(:,1),pos(:,2),pos(:,4),pos(:,5),pos(:,7),pos(:,8),pos(:,10),pos(:,11),pos(:,13),pos(:,14),pos(:,16),pos(:,17))
xlim([0 25]);
ylim([0 25]);


    function dp=robot(t,pos)
        dp=zeros(9,1);        
        x(1:3)=pos(1:3);y(1:3)=pos(4:6);theta(1:3)=pos(7:9);
%         if ((pos(1)-T(1))^2)+((pos(2)-T(2))^2)>1 || ((pos(4)-T(1))^2)+((pos(5)-T(2))^2)>1 || ((pos(7)-T(1))^2)+((pos(8)-T(2))^2)>1 || ((pos(10)-T(1))^2)+((pos(11)-T(2))^2)>1 || ((pos(13)-T(1))^2)+((pos(14)-T(2))^2)>1 || ((pos(16)-T(1))^2)+((pos(17)-T(2))^2)>1
%             km=0.05;
%         else
%             km=0;
%         end
%         for ir=1:1:5
%             if ((pos(1)-pos(3*ir+1))^2)+((pos(1)-pos(3*ir+2))^2)>4
%                 Gf(ir+1)=0;
%             end
%         end
        FTx=kT*(T(1,1)-x(1))/(sqrt(((T(1,1)-x(1))^2)+((T(1,2)-y)^2)));
        Fo1x=-ko(1,1)*(obs(1,1)-x)/((((obs(1,1)-x)^2)+(obs(1,2)-y)^2)^1.5);
        Fo2x=-ko(1,2)*(obs(2,1)-x)/((((obs(2,1)-x)^2)+(obs(2,2)-y)^2)^1.5);
        Fom2x=-km*(x2-x)/((((x2-x)^2)+(y2-y)^2)^1.5);
        Fom3x=-km*(x3-x)/((((x3-x)^2)+(y3-y)^2)^1.5);
        Fom4x=-km*(x4-x)/((((x4-x)^2)+(y4-y)^2)^1.5);
        Fom5x=-km*(x5-x)/((((x5-x)^2)+(y5-y)^2)^1.5);
        Fom6x=-km*(x6-x)/((((x6-x)^2)+(y6-y)^2)^1.5);
        Fx=FTx+Fo1x+Fo2x+Fom2x+Fom3x+Fom4x+Fom5x+Fom6x;
        FTy=kT*(T(1,2)-y)/(sqrt(((T(1,1)-x)^2)+(T(1,2)-y)^2));
        Fo1y=-ko(1,1)*(obs(1,2)-y)/((((obs(1,1)-x)^2)+(obs(1,2)-y)^2)^1.5);
        Fo2y=-ko(1,2)*(obs(2,2)-y)/((((obs(2,1)-x)^2)+(obs(2,2)-y)^2)^1.5);
        Fom2y=-km*(y2-y)/((((x2-x)^2)+(y2-y)^2)^1.5);
        Fom3y=-km*(y3-y)/((((x3-x)^2)+(y3-y)^2)^1.5);
        Fom4y=-km*(y4-y)/((((x4-x)^2)+(y4-y)^2)^1.5);        
        Fom5y=-km*(y5-y)/((((x5-x)^2)+(y5-y)^2)^1.5);       
        Fom6y=-km*(y6-y)/((((x6-x)^2)+(y6-y)^2)^1.5);       
        Fy=FTy+Fo1y+Fo2y+Fom2y+Fom3y+Fom4y+Fom5y+Fom6y;
        thetades=(Gp*atan2(Fy,Fx));
        fi=kn*(thetades-theta);
        
        FTx2=kT*(T(1,1)-x2)/(sqrt(((T(1,1)-x2)^2)+((T(1,2)-y2)^2)));
        Fo1x2=-ko(1,1)*(obs(1,1)-x2)/((((obs(1,1)-x2)^2)+(obs(1,2)-y2)^2)^1.5);
        Fo2x2=-ko(1,2)*(obs(2,1)-x2)/((((obs(2,1)-x2)^2)+(obs(2,2)-y2)^2)^1.5);
        Fom1x2=-km*(x-x2)/((((x-x2)^2)+(y-y2)^2)^1.5);
        Fom3x2=-km*(x3-x2)/((((x3-x2)^2)+(y3-y2)^2)^1.5);
        Fom4x2=-km*(x4-x2)/((((x4-x2)^2)+(y4-y2)^2)^1.5);
        Fom5x2=-km*(x5-x2)/((((x5-x2)^2)+(y5-y2)^2)^1.5);
        Fom6x2=-km*(x6-x2)/((((x6-x2)^2)+(y6-y2)^2)^1.5);
        Fx2=FTx2+Fo1x2+Fo2x2+Fom1x2+Fom3x2+Fom4x2+Fom5x2+Fom6x2;
        FTy2=kT*(T(1,2)-y2)/(sqrt(((T(1,1)-x2)^2)+(T(1,2)-y2)^2));
        Fo1y2=-ko(1,1)*(obs(1,2)-y2)/((((obs(1,1)-x2)^2)+(obs(1,2)-y2)^2)^1.5);
        Fo2y2=-ko(1,2)*(obs(2,2)-y2)/((((obs(2,1)-x2)^2)+(obs(2,2)-y2)^2)^1.5);
        Fom1y2=-km*(y-y2)/((((x-x2)^2)+(y-y2)^2)^1.5);
        Fom3y2=-km*(y3-y2)/((((x3-x2)^2)+(y3-y2)^2)^1.5);
        Fom4y2=-km*(y4-y2)/((((x4-x2)^2)+(y4-y2)^2)^1.5);
        Fom5y2=-km*(y5-y2)/((((x5-x2)^2)+(y5-y2)^2)^1.5);
        Fom6y2=-km*(y6-y2)/((((x6-x2)^2)+(y6-y2)^2)^1.5);
        Fy2=FTy2+Fo1y2+Fo2y2+Fom1y2+Fom3y2+Fom4y2+Fom5y2+Fom6y2;
        thetades2=(Gp*atan2(Fy2,Fx2)+Gf(2)*(theta-theta2));
        fi2=kn*(thetades2-theta2);
        
        FTx3=kT*(T(1,1)-x3)/(sqrt(((T(1,1)-x3)^2)+((T(1,2)-y3)^2)));
        Fo1x3=-ko(1,1)*(obs(1,1)-x3)/((((obs(1,1)-x3)^2)+(obs(1,2)-y3)^2)^1.5);
        Fo2x3=-ko(1,2)*(obs(2,1)-x3)/((((obs(2,1)-x3)^2)+(obs(2,2)-y3)^2)^1.5);
        Fom1x3=-km*(x-x3)/((((x-x3)^2)+(y-y3)^2)^1.5);
        Fom2x3=-km*(x2-x3)/((((x2-x3)^2)+(y2-y3)^2)^1.5);
        Fom4x3=-km*(x4-x3)/((((x4-x3)^2)+(y4-y3)^2)^1.5);
        Fom5x3=-km*(x5-x3)/((((x5-x3)^2)+(y5-y3)^2)^1.5);        
        Fom6x3=-km*(x6-x3)/((((x6-x3)^2)+(y6-y3)^2)^1.5);
        Fx3=FTx3+Fo1x3+Fo2x3+Fom1x3+Fom2x3+Fom4x3+Fom5x3+Fom6x3;
        FTy3=kT*(T(1,2)-y3)/(sqrt(((T(1,1)-x3)^2)+(T(1,2)-y3)^2));
        Fo1y3=-ko(1,1)*(obs(1,2)-y3)/((((obs(1,1)-x3)^2)+(obs(1,2)-y3)^2)^1.5);
        Fo2y3=-ko(1,2)*(obs(2,2)-y3)/((((obs(2,1)-x3)^2)+(obs(2,2)-y3)^2)^1.5);
        Fom1y3=-km*(y-y3)/((((x-x3)^2)+(y-y3)^2)^1.5);
        Fom2y3=-km*(y2-y3)/((((x2-x3)^2)+(y2-y3)^2)^1.5);
        Fom4y3=-km*(y4-y3)/((((x4-x3)^2)+(y4-y3)^2)^1.5);        
        Fom5y3=-km*(y5-y3)/((((x5-x3)^2)+(y5-y3)^2)^1.5); 
        Fom6y3=-km*(y6-y3)/((((x6-x3)^2)+(y6-y3)^2)^1.5);                
        Fy3=FTy3+Fo1y3+Fo2y3+Fom1y3+Fom2y3+Fom4y3+Fom5y3+Fom6y3;
        thetades3=(Gp*atan2(Fy3,Fx3)+Gf(3)*(theta-theta3));
        fi3=kn*(thetades3-theta3);

        FTx4=kT*(T(1,1)-x4)/(sqrt(((T(1,1)-x4)^2)+((T(1,2)-y4)^2)));
        Fo1x4=-ko(1,1)*(obs(1,1)-x4)/((((obs(1,1)-x4)^2)+(obs(1,2)-y4)^2)^1.5);
        Fo2x4=-ko(1,2)*(obs(2,1)-x4)/((((obs(2,1)-x4)^2)+(obs(2,2)-y4)^2)^1.5);
        Fom1x4=-km*(x-x4)/((((x-x4)^2)+(y-y4)^2)^1.5);
        Fom2x4=-km*(x2-x4)/((((x2-x4)^2)+(y2-y4)^2)^1.5);
        Fom3x4=-km*(x3-x4)/((((x3-x4)^2)+(y3-y4)^2)^1.5);
        Fom5x4=-km*(x5-x4)/((((x5-x4)^2)+(y5-y4)^2)^1.5);        
        Fom6x4=-km*(x6-x4)/((((x6-x4)^2)+(y6-y4)^2)^1.5);        
        Fx4=FTx4+Fo1x4+Fo2x4+Fom1x4+Fom2x4+Fom3x4+Fom5x4+Fom6x4;
        FTy4=kT*(T(1,2)-y4)/(sqrt(((T(1,1)-x4)^2)+(T(1,2)-y4)^2));
        Fo1y4=-ko(1,1)*(obs(1,2)-y4)/((((obs(1,1)-x4)^2)+(obs(1,2)-y4)^2)^1.5);
        Fo2y4=-ko(1,2)*(obs(2,2)-y4)/((((obs(2,1)-x4)^2)+(obs(2,2)-y4)^2)^1.5);
        Fom1y4=-km*(y-y4)/((((x-x4)^2)+(y-y4)^2)^1.5);
        Fom2y4=-km*(y2-y4)/((((x2-x4)^2)+(y2-y4)^2)^1.5);
        Fom3y4=-km*(y3-y4)/((((x3-x4)^2)+(y3-y4)^2)^1.5);        
        Fom5y4=-km*(y5-y4)/((((x5-x4)^2)+(y5-y4)^2)^1.5);        
        Fom6y4=-km*(y6-y4)/((((x6-x4)^2)+(y6-y4)^2)^1.5);                
        Fy4=FTy4+Fo1y4+Fo2y4+Fom1y4+Fom2y4+Fom3y4+Fom5y4+Fom6y4;
        thetades4=(Gp*atan2(Fy4,Fx4)+Gf(4)*(theta2-theta4));
        fi4=kn*(thetades4-theta4);

        FTx5=kT*(T(1,1)-x5)/(sqrt(((T(1,1)-x5)^2)+((T(1,2)-y5)^2)));
        Fo1x5=-ko(1,1)*(obs(1,1)-x5)/((((obs(1,1)-x5)^2)+(obs(1,2)-y5)^2)^1.5);
        Fo2x5=-ko(1,2)*(obs(2,1)-x5)/((((obs(2,1)-x5)^2)+(obs(2,2)-y5)^2)^1.5);
        Fom1x5=-km*(x-x5)/((((x-x5)^2)+(y-y5)^2)^1.5);
        Fom2x5=-km*(x2-x5)/((((x2-x5)^2)+(y2-y5)^2)^1.5);
        Fom3x5=-km*(x3-x5)/((((x3-x5)^2)+(y3-y5)^2)^1.5);
        Fom4x5=-km*(x4-x5)/((((x4-x5)^2)+(y4-y5)^2)^1.5);        
        Fom6x5=-km*(x6-x5)/((((x6-x5)^2)+(y6-y5)^2)^1.5);        
        Fx5=FTx5+Fo1x5+Fo2x5+Fom1x5+Fom2x5+Fom3x5+Fom4x5+Fom6x5;
        FTy5=kT*(T(1,2)-y5)/(sqrt(((T(1,1)-x5)^2)+(T(1,2)-y5)^2));
        Fo1y5=-ko(1,1)*(obs(1,2)-y5)/((((obs(1,1)-x5)^2)+(obs(1,2)-y5)^2)^1.5);
        Fo2y5=-ko(1,2)*(obs(2,2)-y5)/((((obs(2,1)-x5)^2)+(obs(2,2)-y5)^2)^1.5);
        Fom1y5=-km*(y-y5)/((((x-x5)^2)+(y-y5)^2)^1.5);
        Fom2y5=-km*(y2-y5)/((((x2-x5)^2)+(y2-y5)^2)^1.5);
        Fom3y5=-km*(y3-y5)/((((x3-x5)^2)+(y3-y5)^2)^1.5);        
        Fom4y5=-km*(y4-y5)/((((x4-x5)^2)+(y4-y5)^2)^1.5);        
        Fom6y5=-km*(y6-y5)/((((x6-x5)^2)+(y6-y5)^2)^1.5);                
        Fy5=FTy5+Fo1y5+Fo2y5+Fom1y5+Fom2y5+Fom3y5+Fom4y5+Fom6y5;
        thetades5=(Gp*atan2(Fy5,Fx5)+Gf(5)*(theta3-theta5));
        fi5=kn*(thetades5-theta5);
        
        FTx6=kT*(T(1,1)-x6)/(sqrt(((T(1,1)-x6)^2)+((T(1,2)-y6)^2)));
        Fo1x6=-ko(1,1)*(obs(1,1)-x6)/((((obs(1,1)-x6)^2)+(obs(1,2)-y6)^2)^1.5);
        Fo2x6=-ko(1,2)*(obs(2,1)-x6)/((((obs(2,1)-x6)^2)+(obs(2,2)-y6)^2)^1.5);
        Fom1x6=-km*(x-x6)/((((x-x6)^2)+(y-y6)^2)^1.5);
        Fom2x6=-km*(x2-x6)/((((x2-x6)^2)+(y2-y6)^2)^1.5);
        Fom3x6=-km*(x3-x6)/((((x3-x6)^2)+(y3-y6)^2)^1.5);
        Fom4x6=-km*(x4-x6)/((((x4-x6)^2)+(y4-y6)^2)^1.5);        
        Fom5x6=-km*(x5-x6)/((((x5-x6)^2)+(y5-y6)^2)^1.5);        
        Fx6=FTx6+Fo1x6+Fo2x6+Fom1x6+Fom2x6+Fom3x6+Fom4x6+Fom5x6;
        FTy6=kT*(T(1,2)-y6)/(sqrt(((T(1,1)-x6)^2)+(T(1,2)-y6)^2));
        Fo1y6=-ko(1,1)*(obs(1,2)-y6)/((((obs(1,1)-x6)^2)+(obs(1,2)-y6)^2)^1.5);
        Fo2y6=-ko(1,2)*(obs(2,2)-y6)/((((obs(2,1)-x6)^2)+(obs(2,2)-y6)^2)^1.5);
        Fom1y6=-km*(y-y6)/((((x-x6)^2)+(y-y6)^2)^1.5);
        Fom2y6=-km*(y2-y6)/((((x2-x6)^2)+(y2-y6)^2)^1.5);
        Fom3y6=-km*(y3-y6)/((((x3-x6)^2)+(y3-y6)^2)^1.5);        
        Fom4y6=-km*(y4-y6)/((((x4-x6)^2)+(y4-y6)^2)^1.5);        
        Fom5y6=-km*(y5-y6)/((((x5-x6)^2)+(y5-y6)^2)^1.5);                
        Fy6=FTy6+Fo1y6+Fo2y6+Fom1y6+Fom2y6+Fom3y6+Fom4y6+Fom5y6;
        thetades6=(Gp*atan2(Fy6,Fx6)+Gf(6)*(theta3-theta6));
        fi6=kn*(thetades6-theta6);
        
        dp(1)=Vp*cos(fi)*cos(theta);
        dp(2)=Vp*cos(fi)*sin(theta);
        dp(3)=(Vp/L)*sin(fi);
        dp(4)=Vp*cos(fi2)*cos(theta2);
        dp(5)=Vp*cos(fi2)*sin(theta2);
        dp(6)=(Vp/L)*sin(fi2);
        dp(7)=Vp*cos(fi3)*cos(theta3);
        dp(8)=Vp*cos(fi3)*sin(theta3);
        dp(9)=(Vp/L)*sin(fi3);
        dp(10)=Vp*cos(fi4)*cos(theta4);
        dp(11)=Vp*cos(fi4)*sin(theta4);
        dp(12)=(Vp/L)*sin(fi4);
        dp(13)=Vp*cos(fi5)*cos(theta5);
        dp(14)=Vp*cos(fi5)*sin(theta5);
        dp(15)=(Vp/L)*sin(fi5);
        dp(16)=Vp*cos(fi6)*cos(theta6);
        dp(17)=Vp*cos(fi6)*sin(theta6);
        dp(18)=(Vp/L)*sin(fi6);
    end
    function [Val,Ister,Dir]=StopSim(t,pos)
        if ((pos(1)-T(1))^2)+((pos(2)-T(2))^2)<1e-2
            Val(1)=0;
        else
            Val(1)=1;
        end
        Ister(1)=1;
        Dir(1)=0;
    end 
end