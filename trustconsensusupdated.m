function trustconsensusupdated
itrust(:,1)=[0;0.5;0.8;0.7;-0.5;0.6];
itrust(:,2)=[0.4;0;0.4;0.6;-0.5;0.9];
itrust(:,3)=[0.7;0.6;0;0.7;0.5;0.6];
itrust(:,4)=[0.9;0.5;0.6;0;-0.5;0.7];
itrust(:,5)=[0.3;0.5;0.6;0.4;0;0.9];
itrust(:,6)=[0.9;0.5;0.8;0.7;0.5;0];
init=[itrust(:,1);itrust(:,2);itrust(:,3);itrust(:,4);itrust(:,5);itrust(:,6)];
A=[0 0 0.5 0 0 0;0.5 0 0 0 0 0.5;0.5 0.5 0 0 0 0;0 0.5 0 0 0 0;0 0 0.5 0 0 0;0 0 0 0.5 0.5 0];
D=[0.5 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 0.5 0 0;0 0 0 0 0.5 0;0 0 0 0 0 1];
MO=D-A
%AT=init
tin=[0 25];
[t,x]=ode23(@equations,tin,init);
function dx=equations(t,x)
 dx=zeros(36,1);
% P=x(:,1)
%  for ax=1:6
%      for bx=1:6
%          K=MO(ax,bx)
%          dx(:,ax)=-K.*x(:,bx)'
%      end
%  end
 dx(1:6)=x(13:18)-x(1:6);
 dx(7:12)=x(1:6)+x(31:36)-x(7:12)-x(7:12);
 dx(13:18)=x(1:6)+x(7:12)-x(13:18)-x(13:18);
 dx(19:24)=x(7:12)-x(19:24);
 dx(25:30)=x(13:18)-x(25:30);
 dx(31:36)=x(19:24)+x(25:30)-x(31:36)-x(31:36);
end
figure()
plot(t,x,'LineWidth',2)
Z1=x(:,1:6);
Z2=x(:,7:12);
Z3=x(:,13:18);
Z4=x(:,19:24);
Z5=x(:,25:30);
Z6=x(:,31:36);
finval=length(Z1(:,1));
trustcon=[Z1(finval,:);Z2(finval,:);Z3(finval,:);Z4(finval,:);Z5(finval,:);Z6(finval,:)];
a=0.5; V=1;
tint=[0 100]; %time interval

for i=1:1:6
    for j=1:1:6
        if trustcon(i,j)<0
            ErrN(i)=j;
        end
    end
end
if range(ErrN)==0
    ErrorNode=ErrN(1);
end
for i=1:1:6
    if A(i,ErrorNode)>0
        ou=1;
        DC(ou)=i;
        ErrorWeight(ou)=A(i,ErrorNode);
        ou=ou+1;
    end
end
for i=1:1:length(DC)
    D(DC(i),DC(i))=D(DC(i),DC(i))-ErrorWeight(i);
end
%% if direction and position of erroneous node is still of interest
init=rand(1,6).*(2*pi);
A(:,ErrorNode)=[0 0 0 0 0 0];
A(ErrorNode,:)=[0 0 0 0 0 0];
D(ErrorNode,ErrorNode)=[0];
L=D-A;
[T,th]=ode23(@equation,tint,[init zeros(1,12)]);
function dth=equation(T,th)
    dth=zeros(18,1);
    dth(1:6)=-L*th(1:6);
    dth(7:12)=cos(th(1:6));
    dth(13:18)=sin(th(1:6));
end
figure()
plot(T,th(:,1:6),'LineWidth',2)
figure()
plot(th(:,7:12),th(:,13:18),'LineWidth',2)
%% if erroneous node is no longer of interest
% init=rand(1,5).*(2*pi);
% A(:,ErrorNode)=[];
% A(ErrorNode,:)=[];
% D(:,ErrorNode)=[];
% D(ErrorNode,:)=[];
% L=D-A;
% [T,th]=ode23(@equation,tint,[init zeros(1,10)]);
% function dth=equation(T,th)
%     dth=zeros(15,1);
%     dth(1:5)=-L*th(1:5);
%     dth(6:10)=cos(th(1:5));
%     dth(11:15)=sin(th(1:5));
% end
% figure()
% plot(T,th(:,1:5),'LineWidth',2)
% figure()
% plot(th(:,6:10),th(:,11:15),'LineWidth',2)
end