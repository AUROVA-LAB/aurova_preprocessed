function state = initState()

sigma_randn = 0.2;
sigma_randn2 = 0.8;
xyz = randn(1, 3) * sigma_randn;
ypr = randn(1, 3) * sigma_randn2;

state = [];
state.stt = [xyz ypr]';
state.cov = eye(length(state.stt));
state.cov(1, 1) = sigma_randn;
state.cov(2, 2) = sigma_randn;
state.cov(3, 3) = sigma_randn;
state.cov(4, 4) = sigma_randn2;
state.cov(5, 5) = sigma_randn2;
state.cov(6, 6) = sigma_randn2;

end