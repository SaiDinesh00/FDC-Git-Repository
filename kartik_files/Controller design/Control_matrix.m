function matrx = Control_matrix(CTs)

parameters;
ct1 = CTs(1);
ct2 = CTs(2);
ct3 = CTs(3);
ct4 = CTs(4);

zz = 1.5 * K * R / sqrt(2);
matrx = [K K K K;
         -zz*sqrt(ct1) zz*sqrt(ct2) -zz*sqrt(ct3) zz*sqrt(ct4);
         K*l K*l -K*l -K*l;
         K*l -K*l -K*l K*l];
end
