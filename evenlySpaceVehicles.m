function pose0 = evenlySpaceVehicles(L0, Vx0, Ts)
h = L0/(Vx0*Ts);
pose0 = (4*h :-h: 0) + 1;
end