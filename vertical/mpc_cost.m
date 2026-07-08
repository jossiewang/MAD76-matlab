function J = mpc_cost(u, x0,N,spline,dt)
% Initialize state and cost
x=x0; 
u_last=u(1:2);
J=0;
dist=0;

% Loop over prediction horizon
Q=[5400.6098,0,0;0,500,0;0,0, 30.1091];
R=[0.1000,0;0,3];
R_der=[0.1000,0;0,5];

for i=1:N-1
    
    idx = 2*i-1;
    u_new = u(idx:idx+1);
    [x_diff,dist] = error_dynamics(spline,x);
    
    J_Q=double(x_diff.'*Q*x_diff);
    J_R=double(u_new.'*R*u_new);
    J_dist=double(-0.5'*dist);
    J=J+J_Q+J_R+J_dist;


    if i>=2
        u_der=u_new-u_last;
        J_R_der=double(u_der.'*R_der*u_der);
        u_last=u_new;
        J=J+J_R_der;
    end
    x=simulate_dynamics_rk4(x, u_new, dt);
    
end


% Terminal cost
% P=100.0000;
% x_fin=1.84;
% J=double(J+P*((dist-x_fin)/x_fin).^2);
end