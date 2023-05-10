function dHdt = tank_system_linear(t, H, a, b, A, V, H0) %linear ode
    dHdt = (b/A)*V-(a/(2*A*sqrt(H0)))*(H-H0)-(a*sqrt(H0))/A; 
end