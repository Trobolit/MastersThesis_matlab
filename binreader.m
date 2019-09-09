%% Below is for the MainLog.bin
binstr = 'MainLog.bin';
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
figure,plot(timestamp,datapoints([MODE Q_WF Q_REF],:));%,'*');
legend('mode','q0','q1','q2','q3','qr0','qr1','qr2','qr3');
xlabel('PX4 timestamp in \mu s');
ylabel('y unit');

%% Below is for rc_and_servosignals.bin
RC_INPUTS = 1:18;
SERVO_SIGNALS = 19:26;

binstr = 'rc_and_servosignals.bin';
[datapoints,timestamp,numberofpoints]=px4_read_binary_file(binstr);
figure,plot(timestamp,datapoints([RC_INPUTS SERVO_SIGNALS],:));%,'*');
legend('data1','data2','data3');
xlabel('PX4 timestamp in \mu s');
ylabel('y unit');


