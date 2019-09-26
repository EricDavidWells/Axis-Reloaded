import ETS2.*
a1 = 1;
E = Rz('q1') * Tx(a1);

a2 = 1;
E = Rz('q1') * Tx(a1) * Rz('q2') * Tx(a2);
