function refs = lqr_build_reference(s, vr, SPLINE, params)

%% simply use the provided version for now (TODO)

%BUILD_REFERENCE Build MPC reference preview using MAD76 SPLINE bus.
%
% s = [s_r1; s_r2]
%
% SPLINE is Simulink bus signal with fields:
%   SPLINE.breakslen
%   SPLINE.points      % 3 x max_points
%   SPLINE.coefs       % piecewise polynomial coefficients
%   SPLINE.segments
%   SPLINE.periodic
%
% params:
%   params.N
%   params.Ta
%   params.v_ref
%   params.Tt

%% Protect against using bus TYPE instead of bus DATA
if isa(SPLINE, 'Simulink.Bus')
    error(['SPLINE is a Simulink.Bus type object, not spline data. ', ...
           'Pass the actual SPLINE bus signal into the MPC block.']);
end

%% Extract current position
s_car = single([s(1); s(2)]);

N  = params.N;
Ta = single(params.Ta);
Tt = single(params.Tt);
% params.v_ref = double(vr);

%% Extract spline bus data
breakslen = int32(SPLINE.breakslen);
points    = single(SPLINE.points);
ppcoefs   = single(SPLINE.coefs);
periodic  = logical(SPLINE.periodic);

%% If spline is invalid, return zero reference
if breakslen <= 1
    refs = empty_refs(N);
    return;
end

x_end = points(1, breakslen);

%% Speed reference over horizon
if isscalar(params.v_ref)
    v_ref = single(params.v_ref) * ones(1, N+1, 'single');
else
    v_ref = single(params.v_ref(:).');
    v_ref = v_ref(1:N+1);
end

%% 1) Use original MAD76 reference generator for current nearest reference
[w0, idx0] = mbc_spline_get_reference( ...
    s_car, ...
    breakslen, ...
    points, ...
    ppcoefs, ...
    periodic, ...
    v_ref(1), ...
    Tt, ...
    int32(1), ...
    breakslen);

x0_star = w0(1);

%% 2) Build MPC preview arc lengths
x_ref = zeros(1, N+1, 'single');
x_ref(1) = x0_star;

for i = 2:N+1
    x_ref(i) = x_ref(i-1) + Ta * v_ref(i-1);
end

if ~periodic
    x_ref = min(max(x_ref, single(0)), x_end);
end

%% 3) Evaluate spline at preview points
[s_ref, sd_ref, ~, ~] = mbc_ppval( ...
    breakslen, ...
    points(1,:), ...
    ppcoefs, ...
    x_ref);

%% 4) Reference yaw angle
psi_ref = atan2(sd_ref(2,:), sd_ref(1,:));

%% 5) Tangent and normal vectors
t_ref = [cos(psi_ref);
         sin(psi_ref)];

n_ref = [-sin(psi_ref);
          cos(psi_ref)];

%% 6) Lookahead curvature
x_hat = x_ref + v_ref * Tt;

if ~periodic
    x_hat = min(max(x_hat, single(0)), x_end);
end

[~, sd_hat, sdd_hat, ~] = mbc_ppval( ...
    breakslen, ...
    points(1,:), ...
    ppcoefs, ...
    x_hat);

s1d  = sd_hat(1,:);
s2d  = sd_hat(2,:);
s1dd = sdd_hat(1,:);
s2dd = sdd_hat(2,:);

den = (s1d.^2 + s2d.^2).^(3/2);
den = max(den, single(1e-9));

kappa_ref = (s1d .* s2dd - s1dd .* s2d) ./ den;

%% 7) nominal delta
delta = atan(params.l*kappa_ref);

%% Pack output as double for fmincon
refs.x      = double(x_ref);
refs.s      = double(s_ref);
refs.psi    = double(psi_ref);
refs.t      = double(t_ref);
refs.n      = double(n_ref);
refs.xhat   = double(x_hat);
refs.kappa  = double(kappa_ref);
refs.v      = double(v_ref);
refs.delta  = double(delta);

refs.w0     = double(w0);
refs.idx0   = double(idx0);

end

%% ========================================================================
function refs = empty_refs(N)

refs.x      = zeros(1, N+1);
refs.s      = zeros(2, N+1);
refs.psi    = zeros(1, N+1);
refs.t      = zeros(2, N+1);
refs.n      = zeros(2, N+1);
refs.xhat   = zeros(1, N+1);
refs.kappa  = zeros(1, N+1);
refs.v      = zeros(1, N+1);
refs.delta  = zeros(1, N+1);

refs.w0     = zeros(5,1);
refs.idx0   = -1;

end