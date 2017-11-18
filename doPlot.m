figure();
hold on;
set(gca,'FontSize',14);
title('Y(t)');
p1=plot(T,Y);
p2=plot(T,Y1);
p3=plot(T,Y2);
p4=plot(T,mOpt(:,2),'--k');

xlabel('t, s');
ylabel('Y, sm');

legend('odometry','camera','distance sensor','filter')

figure();
hold on;
set(gca,'FontSize',14);
title('X(t)');
p1=plot(T,X);
p2=plot(T,X1);
p4=plot(T,mOpt(:,1),'--k');

xlabel('t, s');
ylabel('X, sm');

legend('odometry','camera','filter')

figure();
hold on;
set(gca,'FontSize',14);
title('angle');
p1=plot(T,phi);
p2=plot(T,angle);
p4=plot(T,mOpt(:,3),'--k');

xlabel('t, s');
ylabel('angle, rad');

legend('odometry','camera','filter')

figure();
hold on;
set(gca,'FontSize',14);
title('Trajectory');
p1=plot(X,Y);
p2=plot(X,Y1);
p4=plot(mOpt(:,1),mOpt(:,2),'--k');

xlabel('X, sm');
ylabel('Y, sm');

legend('odometry','camera','filter')