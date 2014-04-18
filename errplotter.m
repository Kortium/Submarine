plot((1:length(err1)).*0.25,err1);
xlabel('Время, сек.','FontSize',16,'FontWeight','bold');
ylabel('Ошибка координаты, м.','FontSize',16,'FontWeight','bold');
grid on;
figure
plot((1:length(err_base1)).*0.25,err_base1);
xlabel('Время, сек.','FontSize',16,'FontWeight','bold');
ylabel('Ошибка координаты, м.','FontSize',16,'FontWeight','bold');
grid on;
figure
err23 = err2;
err23b = err_base2;
err23(1,:) = err23(1,:)+1;
err23b(1,:) = err23b(1,:)+1;
[erra1b,erra2b,erra3b]= quat2angle(err23b');
[erra1,erra2,erra3 ]= quat2angle(err23');
plot((1:length(err1)).*0.25,erra1*(180/pi),'DisplayName','erra1');hold all;plot((1:length(err1)).*0.25,erra2*(180/pi),'DisplayName','erra2');plot((1:length(err1)).*0.25,erra3*(180/pi),'DisplayName','erra3');hold off;
xlabel('Время, сек.','FontSize',16,'FontWeight','bold');
ylabel('Ошибка ориентации, \circ.','FontSize',16,'FontWeight','bold');
grid on;
figure
plot((1:length(err1)).*0.25,erra1b*(180/pi),'DisplayName','erra1');hold all;plot((1:length(err1)).*0.25,erra2b*(180/pi),'DisplayName','erra2');plot((1:length(err1)).*0.25,erra3b*(180/pi),'DisplayName','erra3');hold off;
xlabel('Время, сек.','FontSize',16,'FontWeight','bold');
ylabel('Ошибка ориентации, \circ.','FontSize',16,'FontWeight','bold');
grid on