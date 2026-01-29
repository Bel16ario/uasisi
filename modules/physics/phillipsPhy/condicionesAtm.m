classdef condicionesAtm
    % CONDICIONESATM Clase para calcular propiedades del aire a cierta altitud.
    % Métodos estáticos incluyen cálculos para viscosidad, densidad y temperatura.
    
    methods (Static)
        function mu = calcVisc(h)
            % CALCVISC Calcula la viscosidad del aire a cierta altitud.
            %
            %   MU = CALCVISC(H) computa la viscosidad del aire a una altitud
            %   dada según la fórmula de Sutherland.
            
            % Constantes
            T0 = 288.15; % Temperatura estándar a nivel del mar, K
            S = 110.4;   % Constante de Sutherland, K
            mu0 = 1.716e-5; % Viscosidad de referencia a T0 y a nivel del mar, Pa·s
            
            T = condicionesAtm.calcTemp(h);
            mu = mu0 * (T / T0)^(3/2) * (T0 + S) / (T + S);
        end

        function T = calcTemp(h)
            % CALCTEMP Calcula la temperatura a una altitud dada.
            %
            %   T = CALCTEMP(H) devuelve la temperatura en Kelvin según el
            %   modelo estándar de la atmósfera.
            
            if h <= 11000 % Tropósfera
                T = 288.15 - 0.0065 * h;
            elseif h <= 20000 % Estratósfera baja
                T = 216.65;
            else
                error('Altitud no válida. Solo soporta hasta 20,000 m.');
            end
        end

        function rho = calcDen(h)
            % CALCDEN Calcula la densidad del aire a una altitud dada.
            %
            %   RHO = CALCDEN(H) computa la densidad del aire en kg/m³ usando
            %   el modelo estándar de la atmósfera.

            % Constantes
            P0 = 101325; % Presión estándar a nivel del mar, Pa
            T0 = 288.15; % Temperatura estándar a nivel del mar, K
            L = 0.0065;  % Pendiente de temperatura, K/m
            R = 287.05;  % Constante gaseosa específica para el aire, J/(kg·K)
            g = 9.80665; % Gravedad, m/s^2

            if h <= 11000 % Tropósfera
                T = T0 - L * h;
                P = P0 * (T / T0)^(-g / (R * L));
            elseif h <= 20000 % Estratósfera baja
                T = 216.65;
                P = P0 * 0.22336 * exp(-g * (h - 11000) / (R * T));
            else
                error('Altitud no válida. Solo soporta hasta 20,000 m.');
            end

            rho = P / (R * T);
        end
    end
end
