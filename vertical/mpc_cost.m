function J = mpc_cost(u, x0,N,spline,dt)
% Initialize state and cost
x=x0; 
J=0;
dist=0;

% Loop over prediction horizon
Q=[1,0,0;0,1,0;0,0,0.4];
R=[0.2,0;0,0.2];

for i=1:N-1
    [x_diff,dist] = error_dynamics(spline,x,u,dt);
    u_new=u(:,i);
    J=double(J+(x_diff.'*Q*x_diff+u_new.'*R*u_new));
    
    x=simulate_dynamics_rk4(x, u, dt);
    
end


% Terminal cost
P=4;
x_fin=1.84;
J=double(J+P*(dist-x_fin).^2);
end