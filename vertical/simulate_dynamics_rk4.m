% Simulation function using RK4 integration
function x_next = simulate_dynamics_rk4(x, u, dt)
%implement RK4
k1=continuous_dynamics(x,u);
k2=continuous_dynamics(x+(dt/2)*k1,u);
k3=continuous_dynamics(x+(dt/2)*k2,u);
k4=continuous_dynamics(x+dt*k3,u);

x_next = x+(dt/6)*(k1+2*k2+2*k3+k4); 

end