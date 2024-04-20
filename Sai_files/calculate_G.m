function G = calculate_G(neta)
n0 = neta(1);
n1 = neta(2);
n2 = neta(3);
n3 = neta(4);


G = [-n1 n0 n3 -n2;
     -n2 -n3 n0 n1;
     -n3 n2 -n1 n0];