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

[datapointsf,timestampf,numberofpointsf]=px4_read_binary_file('flying.bin');
timestampf = timestampf./(10^6);

[datapointsh,timestamph,numberofpointsh]=px4_read_binary_file('hover.bin');
timestamph = timestamph./(10^6);

%% inds

%itot = 1:numel(timestamp);
ihov = (timestamph>780+5 & timestamph<797)';
ih1 = find(ihov~=0, 1, 'first');
ifly = (timestampf>873+2 & timestampf<890-4)';
if1 = find(ifly~=0, 1, 'first');
%i1 = (timestamp>850)';
inds = ifly;


%% qlogs fly
figure;
title('flysteps');
ax1 = subplot(2,2,1);
hold on;
plot(timestampf(ifly)-timestampf(if1),datapointsf([Q_WF(1),Q_REF(1)],ifly));%,'*');
plot(timestamph(ihov)-timestamph(ih1),datapointsh([Q_WF(1),Q_REF(1)],ihov));%,'*');
hold off;
legend('q0 f','q0r f', 'q0 h','q0r h');
ax2 = subplot(2,2,2);
hold on;
plot(timestampf(ifly)-timestampf(if1),datapointsf([Q_WF(2),Q_REF(2)],ifly));%,'*');
plot(timestamph(ihov)-timestamph(ih1),datapointsh([Q_WF(2),Q_REF(2)],ihov));%,'*');
hold off;
legend('q1 f','q1r f', 'q1 h','q1r h');
ax3 = subplot(2,2,3);
hold on;
plot(timestampf(ifly)-timestampf(if1),datapointsf([Q_WF(3),Q_REF(3)],ifly));%,'*');
plot(timestamph(ihov)-timestamph(ih1),datapointsh([Q_WF(3),Q_REF(3)],ihov));%,'*');
hold off;
legend('q2 f','q2r f', 'q2 h','q2r h');
ax4 = subplot(2,2,4);
hold on;
plot(timestampf(ifly)-timestampf(if1),datapointsf([Q_WF(4),Q_REF(4)],ifly));%,'*');
plot(timestamph(ihov)-timestamph(ih1),datapointsh([Q_WF(4),Q_REF(4)],ihov));%,'*');
hold off;
legend('q3 f','q3r f', 'q3 h','q3r h');
xlabel('PX4 timestamp in \mu s');
ylabel('y unit');
linkaxes([ax1,ax2,ax3,ax4],'x');


%% Step in q2

[pitch, roll, yaw]      = quat2angle(datapointsf(Q_WF, :)','YXZ');
[pitchr, rollr, yawr]   = quat2angle(datapointsf(Q_REF,:)','YXZ');
[pitchh, rollh, yawh]      = quat2angle(datapointsh(Q_WF, :)','YXZ');
[pitchhr, rollhr, yawhr]   = quat2angle(datapointsh(Q_REF,:)','YXZ');

figure;
ax1 = subplot(2,1,1);
hold on;
plot(timestampf(ifly)-timestampf(if1),pitch(ifly));%,'*');
plot(timestampf(ifly)-timestampf(if1),pitchr(ifly));%,'*');
title('pitch, flying');
hold off;
ax2 = subplot(2,1,2);
hold on;
plot(timestampf(ifly)-timestampf(if1),roll(ifly));%,'*');
plot(timestampf(ifly)-timestampf(if1),rollr(ifly));%,'*');
title('roll, flying');
hold off;


figure;
ax1 = subplot(2,1,1);
hold on;
plot(timestamph(ihov)-timestamph(ih1),pitchh(ihov)-0.2);
plot(timestamph(ihov)-timestamph(ih1),pitchhr(ihov));
title('pitch, hovering');
hold off;
ax2 = subplot(2,1,2);
hold on;
plot(timestamph(ihov)-timestamph(ih1),rollh(ihov));
plot(timestamph(ihov)-timestamph(ih1),rollhr(ihov));
title('roll, hovering');
hold off;


%% timeints for steps

ihov_rollstep   = (timestamph>785.5+0.05 & timestamph<790)';
ihov_pitchstep  = (timestamph>790.5 & timestamph<795)';
ifly_rollstep   = (timestampf>875.5 & timestampf<880)';
ifly_pitchstep  = (timestampf>880.5 & timestampf<885)';

ihr1 = find(ihov_rollstep~=0, 1, 'first');
ihp1 = find(ihov_pitchstep~=0, 1, 'first');
ifr1 = find(ifly_rollstep~=0, 1, 'first');
ifp1 = find(ifly_pitchstep~=0, 1, 'first');

%% spit fig
figure;
ax1 = subplot(2,1,1);
hold on;
plot(timestampf(ifly_pitchstep)-timestampf(ifp1),-pitch(ifly_pitchstep));%,'*');
plot(timestampf(ifly_pitchstep)-timestampf(ifp1),-pitchr(ifly_pitchstep));%,'*');
title('pitch, flying');
hold off;
ax2 = subplot(2,1,2);
hold on;
plot(timestampf(ifly_rollstep )-timestampf(ifr1),-roll(ifly_rollstep));%,'*');
plot(timestampf(ifly_rollstep )-timestampf(ifr1),-rollr(ifly_rollstep));%,'*');
title('roll, flying');
hold off;


figure;
ax1 = subplot(2,1,1);
hold on;
plot(timestamph(ihov_pitchstep)-timestamph(ihp1),pitchh(ihov_pitchstep)-0.3);
plot(timestamph(ihov_pitchstep)-timestamph(ihp1),pitchhr(ihov_pitchstep));
title('pitch, hovering');
hold off;
ax2 = subplot(2,1,2);
hold on;
plot(timestamph(ihov_rollstep)-timestamph(ihr1),rollh(ihov_rollstep));
plot(timestamph(ihov_rollstep)-timestamph(ihr1),rollhr(ihov_rollstep));
title('roll, hovering');
hold off;


%% one fig

figure;
ax1 = subplot(2,1,1);
hold on;
plot(timestampf(ifly_pitchstep)-timestampf(ifp1),-pitch(ifly_pitchstep));%,'*');
plot(timestampf(ifly_pitchstep)-timestampf(ifp1),-pitchr(ifly_pitchstep));%,'*');
plot(timestamph(ihov_pitchstep)-timestamph(ihp1),pitchh(ihov_pitchstep)-0.25);
plot(timestamph(ihov_pitchstep)-timestamph(ihp1),pitchhr(ihov_pitchstep));
legend('flying','flying reference','hovering (adjusted for mechanical trim)','hovering reference');
ylabel('radians');
xlabel('seconds');
title('pitch');
hold off;
ax2 = subplot(2,1,2);
hold on;
plot(timestampf(ifly_rollstep )-timestampf(ifr1),-roll(ifly_rollstep));%,'*');
%plot(timestampf(ifly_rollstep )-timestampf(ifr1),-rollr(ifly_rollstep));%,'*');
plot(timestamph(ihov_rollstep)-timestamph(ihr1),rollh(ihov_rollstep));
plot(timestamph(ihov_rollstep)-timestamph(ihr1),rollhr(ihov_rollstep));
legend('flying','hovering','reference');
xlabel('seconds');
ylabel('radians');
title('roll');
hold off;