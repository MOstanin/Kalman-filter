clear
close all;

load('data.mat')
% simple data fusion by the Mean and standard deviation calculation

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

X2_0=logrobot(1,2)*sin(logrobot(1,3)*pi/180);
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

odometry=[X' Y' phi'];

% set deviation
sigmaCamera=1;
sigmaDist=10;
sigmaAngle=5*pi/180;
sigmaOd=1;
sigmaOdAngle=5*pi/180;
sigmaRobot=1;


% simple data fusion by the Mean and standard deviation calculation
xOpt1(1)=0;
for i=2:N
    
    sigma=sqrt((sigmaCamera^2*sigmaOd^2)/(sigmaCamera^2+sigmaOd^2));
    K1=sigma^2/sigmaOd^2;
    K2=sigma^2/sigmaCamera^2;
    xOpt(i)=(X(i))*K1+K2*X1(i);
end

yOpt1(1)=0;
for i=2:N
    if Y2(i)>50    
       sigmaDist= 10; 
    else
        sigmaDist= 255; 
    end
%     1 sensor
    sigma=sqrt((sigmaCamera^2*sigmaOd^2)/(sigmaCamera^2+sigmaOd^2));
    K1=sigma^2/sigmaOd^2;
    K2=sigma^2/sigmaCamera^2;
    yOpt(i)=(Y(i))*K1+K2*Y1(i);
%     2 sensor
    sigma2=sqrt((sigmaDist^2*sigma^2)/(sigmaDist^2+sigma^2));
    K1=sigma^2/sigma^2;
    K2=sigma^2/sigmaDist^2;
    yOpt(i)=(yOpt(i))*K1+K2*Y2(i);
end

aOpt(1)=0;
for i=2:N
    
    sigma=sqrt((sigmaAngle^2*sigmaOdAngle^2)/(sigmaAngle^2+sigmaOdAngle^2));
    K1=sigma^2/sigmaOdAngle^2;
    K2=sigma^2/sigmaAngle^2;
    aOpt(i)=phi(i)*K1+K2*angle(i);
end

mOpt(:,1)=xOpt;
mOpt(:,2)=yOpt;
mOpt(:,3)=aOpt;
doPlot;


