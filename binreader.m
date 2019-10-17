%% names
%{
MainLog.bin
rc_and_servosignals.bin
qlogs.bin
norm_sigs.bin
torques.bin
volt_stop.bin
%}

%% Below is for the MainLog.bin
binstr = 'MainLog.bin';
%binstr = 'quat-rates.bin';
MODE = 1;
Q_WF = 2:5;
POS = 6:8;
Q_REF = 9:12;
GPS_INFO = 9:21;
    FIX_TYPE = 9;
    SATS_USED = 10;
    LAT = 11;
    LONG = 12;
    ALT = 13;
    HDOP = 14;
    VDOP = 15;
    NOISE_PER_MS = 16;
    JAMMING_INDICATOR = 17;
    VEL = 18;
    VEL_N = 19;
    VEL_E = 20;
    VEL_D = 21;
PID_SIGNALS = 22:37;
    PITCH_kPID = 22:24;
    ROLL_kPID = 25:27;
    E_ROLL = 28;
    E_PITCH = 29;
    AUTO_ROLL = 30:35;
        VEC2LINE = 30:32;
        ERROR_HEADING = 33;
        ROLL_B4_SAT = 34;
        ROLL_POST_SAT = 35;
    AUTO_PITCH = 36:37;
        PITCH_B4_SAT = 36;
        PITCH_POST_SAT = 37;

[datapoints,timestamp,numberofpoints]=px4_read_binary_file(binstr);
timestamp = timestamp./(10^6);
figure,plot(timestamp,datapoints(1:end-1,:));%,'*');
legend('wroll','wpitch','wyaw','q0','q1','q2','q3');
xlabel('PX4 timestamp in \mu s');
ylabel('y unit');

%% Below is for rc_and_servosignals.bin
RC_INPUTS = 1:18;
SERVO_SIGNALS = 19:26;

binstr = 'rc_and_servosignals.bin';
[datapoints,timestamp,numberofpoints]=px4_read_binary_file(binstr);
timestamp = timestamp./(10^6);

figure,plot(timestamp,datapoints([RC_INPUTS(1:4)],:));%,'*');
legend('thrust','Ail','Ele','rud');
xlabel('PX4 timestamp in \mu s');
ylabel('y unit');

rud = datapoints(RC_INPUTS(4),:);

%% Below is for qlogs
q = 1:4;
qr = 5:8;

binstr = 'qlogs.bin';
[datapoints,timestamp,numberofpoints]=px4_read_binary_file(binstr);
binstr = 'torques.bin';
[datapointsT,timestampT,numberofpointsT]=px4_read_binary_file(binstr);
timestampT = timestampT./(10^6);
timestamp = timestamp./(10^6);
figure;
ax1 = subplot(2,2,1);
plot(timestamp,datapoints([1,5],:));%,'*');
legend('q0','q0r');
ax2 = subplot(2,2,2);
hold on;
plot(timestamp,datapoints([2,6],:));%,'*');
plot(timestampT,datapointsT(4,:));%,'*');
hold off;
legend('q1','q1r','xe');
ax3 = subplot(2,2,3);
hold on;
plot(timestamp,datapoints([3,7],:));%,'*');
plot(timestampT,datapointsT(5,:));%,'*');
hold off;
legend('q2','q2r','ye');
ax4 = subplot(2,2,4);
hold on;
plot(timestamp,datapoints([4,8],:));%,'*');
plot(timestampT,datapointsT(6,:));%,'*');
hold off;
legend('q3','q3r','ze');
xlabel('PX4 timestamp in \mu s');
ylabel('y unit');
linkaxes([ax1,ax2,ax3,ax4],'x');

%% Below is for norm_sigs.bin
ELEL = 1;
ELER = 2;
TL = 3;
TR = 4;

itot = 1:numel(timestamp);
i0 = (timestamp>63 & timestamp<74)';
i1 = (timestamp>209 & timestamp<215)';
i2 = (timestamp>316 & timestamp<325)';
i5 = (timestamp>96 & timestamp<106)';
inds = itot;

binstr = 'norm_sigs.bin';
[datapoints,timestamp,numberofpoints]=px4_read_binary_file(binstr);
timestamp = timestamp./(10^6);
figure;
ax1 = subplot(2,1,1);
plot(timestamp(inds),datapoints(1:2,inds));%,'*');
legend('EleL','EleR');
ax2 = subplot(2,1,2);
plot(timestamp(inds),datapoints(3:4,inds));%,'*');
legend('TL','TR');
xlabel('PX4 timestamp in \mu s');
ylabel('y unit');
linkaxes([ax1, ax2],'x')

%% Below is for torques.bin

binstr = 'torques.bin';
[datapoints,timestamp,numberofpoints]=px4_read_binary_file(binstr);
timestamp = timestamp./(10^6);
figure;
ax1 = subplot(2,1,1);
plot(timestamp,datapoints(1:3,:));%,'*');
legend('tx','ty','tz');
ax2 = subplot(2,1,2);
plot(timestamp,datapoints(4:6,:));%,'*');
legend('xe','ye','ze');

xlabel('PX4 timestamp in \mu s');
ylabel('y unit');

linkaxes([ax1,ax2],'x');
ze = datapoints(6,:);
tz = datapoints(3,:);

%% yaw debug
figure;
plot(timestamp, ze);
hold on;
plot(timestamp,(rud-1500)./500);
plot(timestamp,tz);
hold off;
legend('ze','rud','tz');


%% Below is for volt_stop
V = 1;
STOP = 2;

binstr = 'volt_stop.bin';
[datapoints,timestamp,numberofpoints]=px4_read_binary_file(binstr);
timestamp = timestamp./(10^6);
figure;
ax1 = subplot(2,1,1);
hold on;
plot(timestamp,datapoints(1,:),'*');
plot(timestamp,datapoints(2,:));%,'*');
legend('volt filtered','volt');
hold off;
ax2 = subplot(2,1,2);
plot(timestamp,datapoints(3,:));%,'*');
legend('stop');
xlabel('PX4 timestamp in \mu s');
ylabel('y unit');
linkaxes([ax1, ax2],'x')

%% volt_stop, actual frequency run at
figure();
plot(timestamp,datapoints(4,:),'*');
