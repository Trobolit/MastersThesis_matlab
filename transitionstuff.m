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

[datapointsf,timestampf,numberofpointsf]=px4_read_binary_file('transition.bin');
timestampf = timestampf./(10^6);

[datapointsh,timestamph,numberofpointsh]=px4_read_binary_file('hover.bin');
timestamph = timestamph./(10^6);


[yaw, pitch, roll]      = quat2angle(datapointsf(Q_WF, :)','ZYX');
[yawr, pitchr, rollr]   = quat2angle(datapointsf(Q_REF,:)','ZYX');
[yawh, pitchh, rollh ]      = quat2angle(datapointsh(Q_WF, :)','ZYX');
[yawhr, pitchhr, rollhr]   = quat2angle(datapointsh(Q_REF,:)','ZYX');


figure;
ax1 = subplot(3,1,1);
plot(timestampf, [pitch,pitchr]);
legend('pitch','pitch reference');
ax2 = subplot(3,1,2);
plot(timestampf, [roll,rollr]);
legend('roll','roll reference');
ax3 = subplot(3,1,3);
plot(timestampf, [yaw,yawr]);
legend('yaw','yaw reference');
linkaxes([ax1,ax2,ax3],'x');
title('flying');

%%
figure;
ax1 = subplot(3,1,1);
plot(timestamph, [pitchh,pitchhr]);
legend('pitch','pitch reference');
ax2 = subplot(3,1,2);
plot(timestamph, [rollh,rollhr]);
legend('roll','roll reference');
ax3 = subplot(3,1,3);
plot(timestamph, [yawh,yawhr]);
legend('yaw','yaw reference');
linkaxes([ax1,ax2,ax3],'x');
title('hovering');

%% qlogs fly
ifly = 1:numel(timestampf);
ihov = 1:numel(timestamph);

figure;
title('flysteps');
ax1 = subplot(2,2,1);
hold on;
plot(timestampf(ifly),datapointsf([Q_WF(1),Q_REF(1)],ifly));%,'*');
plot(timestamph(ihov),datapointsh([Q_WF(1),Q_REF(1)],ihov));%,'*');
hold off;
legend('q0 f','q0r f', 'q0 h','q0r h');
ax2 = subplot(2,2,2);
hold on;
plot(timestampf(ifly),datapointsf([Q_WF(2),Q_REF(2)],ifly));%,'*');
plot(timestamph(ihov),datapointsh([Q_WF(2),Q_REF(2)],ihov));%,'*');
hold off;
legend('q1 f','q1r f', 'q1 h','q1r h');
ax3 = subplot(2,2,3);
hold on;
plot(timestampf(ifly),datapointsf([Q_WF(3),Q_REF(3)],ifly));%,'*');
plot(timestamph(ihov),datapointsh([Q_WF(3),Q_REF(3)],ihov));%,'*');
hold off;
legend('q2 f','q2r f', 'q2 h','q2r h');
ax4 = subplot(2,2,4);
hold on;
plot(timestampf(ifly),datapointsf([Q_WF(4),Q_REF(4)],ifly));%,'*');
plot(timestamph(ihov),datapointsh([Q_WF(4),Q_REF(4)],ihov));%,'*');
hold off;
legend('q3 f','q3r f', 'q3 h','q3r h');
xlabel('PX4 timestamp in \mu s');
ylabel('y unit');
linkaxes([ax1,ax2,ax3,ax4],'x');