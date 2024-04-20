function F_M_ts = control_allocation(CTs)

parameters;
Thrust_coeffs = CTs;
Thrust_coeffs(Thrust_coeffs > 0.6) = 0.6;
Thrust_coeffs(Thrust_coeffs < 0.000001) = 0.000001;
CTs = Thrust_coeffs;
ct1 = CTs(1);
ct2 = CTs(2);
ct3 = CTs(3);
ct4 = CTs(4);

T_t = K * (ct1 + ct2 + ct3 + ct4);
L_t = -K * R * (ct1^1.5 - ct2^1.5 + ct3^1.5 - ct4^1.5) / sqrt(2);
M_t = K * l * (ct1 + ct2 - ct3 - ct4);
N_t = K * l * (ct1 - ct2 - ct3 + ct4);

F_M_ts = [T_t; L_t; M_t; N_t];
end