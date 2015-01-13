
function mymean= plots(filename)
end_value=681;
%plot(1:end_value,filename(5,1:end_value),'b');
figure();
hold on;
% plot(0.6-filename(1,1:end_value),filename(5,1:end_value),'g');
% plot(0.6-filename(1,1:end_value),filename(6,1:end_value),'r');
% plot(0.6-filename(1,1:end_value),filename(7,1:end_value),'c');
grid on;
plot(1:end_value,filename(4,1:end_value),'g');
plot(1:end_value,filename(5,1:end_value),'r');
plot(1:end_value,filename(6,1:end_value),'c');
plot(1:end_value,filename(7,1:end_value),'--b');
plot(1:end_value,filename(8,1:end_value),'--g');
plot(1:end_value,filename(9,1:end_value),'--r');
plot(1:end_value,filename(10,1:end_value),'--c');
hold off;

% mymean=[mean(filename(4,1:end_value)) mean(filename(5,1:end_value)) mean(filename(6,1:end_value)) mean(filename(7,1:end_value)) mean(filename(8,1:end_value)) mean(filename(9,1:end_value)) mean(filename(10,1:end_value)) mean(filename(11,1:end_value))]; 
% figure();
% hold on
% algorithm_1=min(filename(3:11,1:end_value));
% [M,I] = min(filename(3:11,1:end_value))
% cm=sum(filename(3:11,1:end_value).*(0:7))/sum(filename(3:11,1:end_value));
% plot(1:end_value,M(1:end_value),'--r');
% plot(1:end_value,cm(1:end_value),'--c');
% hold off
mymean=2.0;
end