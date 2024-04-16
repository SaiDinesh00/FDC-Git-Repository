function CL = Lift_coefficient(CL_0, CL_alp, alp, alp_0, M_0)

sigma = (1 + exp(-M_0 * (alp - alp_0)) + exp(M_0 * (alp + alp_0))) / ...
        ((1 + exp(-M_0 * (alp - alp_0))) * (1 + exp(M_0 * (alp + alp_0))));
CL = (1 - sigma) * (CL_0 + CL_alp * alp) + ...
      sigma * (2 * sign(alp_0) * cos(alp) * (sin(alp)^2));

end