function ddqo   = DirectDynamics(Tau,qi,dqi)
    ddqo = zeros(1,6);
    c = zeros(6,1);
    m = zeros(6,6);
    g = zeros(6,1);
    [m,g,c] = SolveSyms(qi,dqi);
    ddqo = m\((Tau)-c-g);
end