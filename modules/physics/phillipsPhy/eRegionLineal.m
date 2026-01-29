function [m, b] = eRegionLineal(x, y, umbral)

    % Remove any NaN values
    validIdx = ~isnan(x) & ~isnan(y);
    x = x(validIdx);
    y = y(validIdx);
    
    D = gradient(y, x);
    
    D2 = gradient(D, x);
    
    rLineal = abs(D2) < umbral;
    
    rLinealPadded = [false; rLineal; false];
    tr = diff(rLinealPadded);
    inicio = find(tr == 1);
    final = find(tr == -1) - 1;
    
    if isempty(inicio)
        warning('Ninguna región encontrada. Utilizando todo el dominio.');
        linearIdx = 1:length(x);
    else
        lengths = final - inicio + 1;
        [~, maxIdx] = max(lengths);
        linearIdx = inicio(maxIdx):final(maxIdx);
    end
    
    xLinear = x(linearIdx);
    yLinear = y(linearIdx);
    
    linearCoeffs = polyfit(xLinear, yLinear, 1);
    m = linearCoeffs(1);
    b = linearCoeffs(2);
end