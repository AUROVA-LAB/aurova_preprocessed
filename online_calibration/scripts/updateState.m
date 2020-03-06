function state = updateState(observation, state)

y = observation.obs;
R = observation.cov;

x = state.stt;
P = state.cov;
e = x;

H = eye(length(y));
I = eye(length(y));

% innovation
z = y - e;
Z = H * P * H' + R;

% kalman gain
K = P * H' * inv(Z);

% state correction
state.stt = x + K * z;
state.cov = (I - K * H) * P;

end