%% Load data
binstr = 'torques.bin';
[datapoints,timestamp,numberofpoints]=px4_read_binary_file(binstr);
timestamp = timestamp./(10^6);
ze = datapoints(6,:)';
tz = datapoints(3,:)';
tx = datapoints(1,:)';
ty = datapoints(2,:)';

% SIgnals are saturated at +-1, so show that in plots
ty(abs(tx)>1) = sign(tx(abs(tx)>1));
ty(abs(ty)>1) = sign(ty(abs(ty)>1));
ty(abs(tz)>1) = sign(tz(abs(tz)>1));

binstr = 'rc_and_servosignals.bin';
[datapoints,timestamp,numberofpoints]=px4_read_binary_file(binstr);
timestamp = timestamp./(10^6);
rud = datapoints(4,:);

binstr = 'qlogs.bin';
[datapoints,timestamp,numberofpoints]=px4_read_binary_file(binstr);
timestamp = timestamp./(10^6);
q = datapoints(1:4,:);
qr = datapoints(5:8,:);
%%

[pitch, roll, yaw] = quat2angle(q','YXZ');
[pitchr, rollr, yawr] = quat2angle(qr','YXZ');

%[pitch, yaw, roll] = quat2angle(q','YZX');
%[pitchr,yawr, rollr] = quat2angle(qr','YZX');

i0 = (timestamp>98 & timestamp<105)';
%i0 = (timestamp>63 & timestamp<74)';
%i1 = (timestamp>209 & timestamp<215)';
%i2 = (timestamp>316 & timestamp<325)';
%inds = (i0 | i1);
inds = i0;
plot(timestamp(inds),[rad2deg([yaw(inds),yawr(inds)]),tz(inds).*100]);
legend('yaw','yawr','100*tz');
figure();
plot(timestamp(inds),[rad2deg([roll(inds),rollr(inds)]),tx(inds)*100]);
legend('roll','rollr','tx');
figure();
plot(timestamp(inds),[rad2deg([pitch(inds),pitchr(inds)]),ty(inds)*100]);
legend('pitch','pitchr','ty');