function Fij = calcFij(b, alfa0, c, j, phi)
%CALCFIJ Calcula FIJ
    Fij = ((4*b)/(alfa0*c) + (j)/(sin(phi)))*sin(j*phi);
end
