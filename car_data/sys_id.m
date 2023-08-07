
% for i = 80:5:130
i = 100;
file = "accel_" + string(i) + "_50";
accel = readtable(file);
accel = [accel.Var3 accel.Var4 accel.Var5 accel.Var2];
bias_init = mean(accel(1:30,1));
accel(:,1) = (accel(:,1)-bias_init)*9.81;
integ = accel(2:end,1).*(accel(2:end,4)-accel(1:end-1,4));
vel = cumsum(integ);
figure()
plot(accel(:,1))
file2 = "brake_list_" + string(i) + "_50";
brake = readtable(file2);
brake = brake.Var2;
figure()
plot(brake)
file3 = "pos_list_" + string(i) + "_0";
pos = readtable(file3);
pos = [pos.Var2 pos.Var3];
vel2 = (pos(21:end, 1) - pos(20:end-1, 1))*1000./(pos(21:end, 2) - pos(20:end-1, 2));
% end
%     figure()
%     plot(pos(21:end,2), vel2)
t0 = 532;
tf = 650;
vel_sysid = vel(t0:tf);
pwm_sysid = 100*ones(tf-t0+1,1);
data = iddata(vel_sysid, pwm_sysid, 0.01);
% sys = tfest(data,3);

%%
throttle_sys = tf(0.4064, [1, 2.311, 41.36, 45.4]);
Kp = 100;
Ki = 2;
Kd = 20;
PID = tf([Kd, Kp, Ki], 1);
rlocus(PID*throttle_sys)
feedback_sys = feedback(throttle_sys*PID, -1);
step(100*throttle_sys)

