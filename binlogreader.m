binstr = 'log.bin';
[datapoints,timestamp,numberofpoints]=px4_read_binary_file(binstr);
timestamp = timestamp./(10^6);

RC_IN = 1:18;
MODE = 19;
POS = 20:22;
V_BF = 23:25;
NORM_SIGS = 26:29;
Q_WF = 30:33;
Q_REF = 34:37;
TAU = 38:40;
AXIS_ERR = 41:43;
STOP = 44;

figure,plot(timestamp,datapoints([1:4,7,9],:));%,'*');
legend('thrust','Ail','Ele','rud','sa:mode','sc:step');
xlabel('PX4 timestamp in \mu s');
ylabel('y unit');

%% inds

itot = 1:numel(timestamp);
i0 = (timestamp>132 & timestamp<144)';
i1 = (timestamp>560)';
inds = itot;

%% qlogs

figure;
ax1 = subplot(2,2,1);
plot(timestamp(inds),datapoints([Q_WF(1),Q_REF(1)],inds));%,'*');
legend('q0','q0r');
ax2 = subplot(2,2,2);
hold on;
plot(timestamp(inds),datapoints([Q_WF(2),Q_REF(2)],inds));%,'*');
plot(timestamp(inds),datapoints(AXIS_ERR(1),inds));%,'*');
hold off;
legend('q1','q1r','xe');
ax3 = subplot(2,2,3);
hold on;
plot(timestamp(inds),datapoints([Q_WF(3),Q_REF(3)],inds));%,'*');
plot(timestamp(inds),datapoints(AXIS_ERR(2),inds));%,'*');
hold off;
legend('q2','q2r','ye');
ax4 = subplot(2,2,4);
hold on;
plot(timestamp(inds),datapoints([Q_WF(4),Q_REF(4)],inds));%,'*');
plot(timestamp(inds),datapoints(AXIS_ERR(3),inds));%,'*');
hold off;
legend('q3','q3r','ze');
xlabel('PX4 timestamp in \mu s');
ylabel('y unit');
linkaxes([ax1,ax2,ax3,ax4],'x');

%% norm_sigs
figure;
ax1 = subplot(2,1,1);
plot(timestamp(inds),datapoints(NORM_SIGS(1:2),inds));%,'*');
legend('EleL','EleR');
ax2 = subplot(2,1,2);
plot(timestamp(inds),datapoints(NORM_SIGS(3:4),inds));%,'*');
legend('TL','TR');
xlabel('PX4 timestamp in \mu s');
ylabel('y unit');
linkaxes([ax1, ax2],'x')

%% v_bf
figure;
plot(timestamp(inds),datapoints(V_BF,inds));%,'*');
legend('vx','vy','vz');
xlabel('PX4 timestamp in \mu s');
ylabel('y unit');

%% batt
figure;
plot(timestamp(inds),datapoints(STOP,inds));%,'*');
legend('stop');
xlabel('PX4 timestamp in \mu s');
ylabel('y unit');
