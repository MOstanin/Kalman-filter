clear
close all;

load('data.mat')

% comlex KF 

% logcamera {'time' 'X' 'Y'}
% logrobot {'time' 'distance' 'alngle' 'left_speed' 'rigth_speed'}

N1=size(logcamera);
N2=size(logrobot);

T0=min([logcamera(1,1) logrobot(1,1)]);
k=(logcamera(N1(1),1)-T0)/(logrobot(N2(1),1)-T0);

c=1;
for i=1:3:N1(1)
    
    X1(c)=mean([logcamera(i,2), logcamera(i+1,2)]);
    Y1(c)=mean([logcamera(i,3), logcamera(i+1,3)]);
    T1(c)=(logcamera(i,1)-T0)/k;
    
    c=c+1;
    
    X1(c)=mean([logcamera(i+1,2), logcamera(i+2,2)]);
    Y1(c)=mean([logcamera(i+1,3), logcamera(i+2,3)]);
    T1(c)=(logcamera(i,1)-T0)/k;
    c=c+1;
    
end

N2=size(X1);
N=N2(2);

for i=2:N
    Vx1(i)=(X1(i)-X1(i-1))/(T1(i)-T1(i-1));
    Vy1(i)=(Y1(i)-Y1(i-1))/(T1(i)-T1(i-1));
end

Y2_0=logrobot(1,2)*cos(logrobot(1,3)*pi/180);

for i=1:N
    angle(i)=logrobot(i,3)*pi/180;
    Y2(i)=logrobot(i,2)*cos(logrobot(i,3)*pi/180)-Y2_0;
    T(i)=logrobot(i,1)-T0;
end

for i=2:N
    Vy2(i)=(Y2(i)-Y2(i-1))/(T(i)-T(i-1));
end

X(1)=0;
Y(1)=0;
phi(1)=0;
R=2.7;
l=15;
for i=2:N
    wr=logrobot(i,4)/180*pi;
    wl=logrobot(i,5)/180*pi;
    dt=(T(i)-T(i-1));
    Lr=wr*dt*R;
    Ll=wl*dt*R;
    phi(i)=phi(i-1)+(Lr-Ll)/l;
    L=(Lr+Ll)/2;
    X(i)=X(i-1)+L*cos(phi(i));
    Y(i)=Y(i-1)+L*sin(phi(i));
    w(i)=(Lr-Ll)/l;
    Vx(i)=L*cos(phi(i));
    Vy(i)=L*sin(phi(i));
end


sigmaCamera=1;
sigmaDist=10;
sigmaAngle=5*pi/180;
sigmaOd=0.1;
sigmaOdAngle=5*pi/180;
sigmaRobot=1;

% KF 

mOpt(1,:)=[0 0 0]';
P(1,1:3,1:3)=sigmaRobot;
Q=[0.1 0 0; 0 0.1 0; 0 0 0.1];
G=[1 0 0;...
   0 1 0;...
   0 0 1;];

H=[ 1 0 0;...
    0 0 0;...
    0 1 0;...
    0 1 0;...
    0 0 0;...
    0 0 1;];
for i=2:N
    if Y2(i)>50       
        R=[sigmaCamera 0 0 0 0 0;...
           0 15 0 0 0 0;...
           0 0 sigmaCamera 0 0 0;...
           0 0 0 15 0 0;...
           0 0 0 0 inf 0;...
           0 0 0 0 0 sigmaAngle;]; 
    else
        R=[sigmaCamera 0 0 0 0 0;...
           0 inf 0 0 0 0;...
           0 0 sigmaCamera 0 0 0;...
           0 0 0 inf 0 0;...
           0 0 0 0 inf 0;...
           0 0 0 0 0 sigmaAngle;]; 
    end
    G=[1 0 -sin(phi(i));...
       0 1 cos(phi(i));...
       0 0 1];
    mPredict=mOpt(i-1,:)'+[Vx(i) Vy(i) w(i)]';
    Pk=G*[P(i-1,1,1), P(i-1,1,2), P(i-1,1,3);...
        P(i-1,2,1), P(i-1,2,2), P(i-1,2,3);...
        P(i-1,3,1), P(i-1,3,2), P(i-1,3,3)]*G+Q;
    K=Pk*H'*inv(H*Pk*H'+R);
    Z=[X1(i) 0 Y1(i) Y2(i) 0 angle(i)]';
    KM=K*(Z-H*mPredict);
    mOpt(i,:)=mPredict+KM;
    P(i,1:3,1:3)=(eye(3)-K*H)*Pk;
    
end

doPlot;