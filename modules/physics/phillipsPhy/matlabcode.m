classdef modSF < matlab.System
    % Modelo de comportamiento físico
    % Inputs: gAlfaDiscNorm (Matriz Nx2 - VARIABLE SIZE)
    % Outputs: l (distribución de sustentación), L (sustentación total), Q (momento de alabeo), z (Dominio)
    
    properties (Nontunable)
        b = 6;              % envergadura [m]
        c = 1;              % cuerda [m]
        
        vinf = 166;         % velocidad [m/s]
        h = 1000;           % altitud [m]
        alfa0 = 0;          % ángulo de ataque base [rad]
        Omega = -5*pi/180;  % máxima torsión geométrica [rad]
        
        airfoil = 'NACA 2412';
        rAlfa = -20:0.05:20; % resolución de polar
        
        N = 151;            % Número de estaciones
        
        umbral = 0.001;
        
        xfoilPath = 'F:\XFOIL\xfoil.exe';
        dirPolares = 'polares';
    end
    
    properties (Access = private)
        % Variables computadas en la inicialización
        Phi             % Estaciones en coordenadas angulares
        z               % Estaciones en coordenadas cartesianas
        gAlfaNorm       % Configuración alar normalizada
        Fij             % Matriz de influencias
        invFij          % Inverso de Matriz de Influencias
        rho0            % Densidad del aire
        mu0             % Viscosidad dinámica
        dPhi            % Distancia angular entre estaciones
        Re              % Número de Reynolds
        
        % Valores obtenidos durante inicialización con los polares
        pAlfa0          % Ángulo de nula sustentación [rad]
        pM              % Pendiente de coeficiente de sustentación [1/rad]
    end
    
    methods (Access = protected)
        
        function setupImpl(obj, gAlfaDiscNorm)
            % Inicialización
            
            % Condiciones atmosféricas
            obj.rho0 = condicionesAtm.calcDen(obj.h);
            obj.mu0 = condicionesAtm.calcVisc(obj.h);
            
            % Número de Reynolds
            obj.Re = (obj.rho0 * obj.vinf * obj.c) / obj.mu0;
            
            % Obtener polares
            obj.oDPolares();
            
            % Generar dominio discretizado
            obj.initDom();
            
            % Obtener la matriz de influencias
            obj.calMatInf();
        end
        
        function [l, L, Q, zN, Fij] = stepImpl(obj, gAlfaDiscNorm)
            % Código ejecutado cada paso de la simulación
            
            % Ensure gAlfaDiscNorm is properly shaped
            if size(gAlfaDiscNorm, 2) ~= 2
                error('gAlfaDiscNorm must have exactly 2 columns [position, angle]');
            end
            
            % Procesar la configuración alar - FIXED: Remove duplicates before interpolation
            gAlfaNorm = obj.gAlfaNorm;
            
            % Remove duplicate positions with tolerance
            [~, idx] = uniquetol(gAlfaDiscNorm(:,1), 1e-10, 'DataScale', 1);
            gAlfaDiscNormLimpia = gAlfaDiscNorm(idx, :);
            
            % Interpolate with cleaned data
            if size(gAlfaDiscNormLimpia, 1) > 1
                gAlfaNorm(:,2) = interp1(gAlfaDiscNormLimpia(:,1), gAlfaDiscNormLimpia(:,2), ...
                                       gAlfaNorm(:,1), 'linear', 'extrap');
            else
                gAlfaNorm(:,2) = gAlfaDiscNormLimpia(1,2);
            end
            
            omega = gAlfaNorm(:,2);
            
            % Obtener coeficientes de Fourier
            aj = obj.invFij * ones(obj.N-1, 1);
            bj = obj.invFij * omega;
            
            % Para estabilidad numérica
            aj(abs(aj) < 1e-10) = 0;
            
            % Coeficientes totales
            Aj = aj * (obj.alfa0 - obj.pAlfa0) - bj * obj.Omega;
            
            % Distribución de sustentación
            l = zeros(obj.N-1, 1);
            for i = 1:(obj.N-1)
                sum_val = 0;
                for j = 1:(obj.N-1)
                    sum_val = sum_val + Aj(j) * sin(j * obj.Phi(i));
                end
                l(i) = 2 * obj.rho0 * obj.vinf^2 * obj.b * sum_val;
            end
            
            % Valores totales
            [z_ord, idx] = sort(obj.z);
            l_ord = l(idx);
            L = trapz(z_ord, l_ord);
            Q = -trapz(z_ord, l_ord .* z_ord);
            zN = 2*obj.z / obj.b;
            Fij = obj.Fij;
        end
        
        function [sz_1, sz_2, sz_3, sz_4, sz_5] = getOutputSizeImpl(obj)
            % Dimensiones de salidas
            sz_1 = [obj.N-1, 1];  % l (Distribución de sustentación)
            sz_2 = [1, 1];        % L (Sustentación total)
            sz_3 = [1, 1];        % Q (Momento de alabeo)
            sz_4 = [obj.N-1, 1];  % z (Coordenadas a lo largo de la ala)
            sz_5 = [obj.N-1, obj.N-1]; %Matriz de Influencias
        end
        
        function sz = getInputSizeImpl(~)
            % FIXED: Allow variable size input
            % The input gAlfaDiscNorm can have variable number of rows but must have 2 columns
            sz = [-1, 2];  % Variable number of rows, exactly 2 columns
        end

        function dt = getInputDataTypeImpl(~)
            dt = 'double';
        end
        
        function cp = isInputComplexImpl(~)
            cp = false;
        end
        
        function fp = isInputFixedSizeImpl(~)
            % FIXED: Input is now variable size
            fp = false;  % Changed from true to false
        end
        
        function flag = supportsMultipleInstanceImpl(~)
            % Support multiple instances
            flag = true;
        end
        
        function [inputSizes] = getInputSizesImpl(~)
            % Explicitly define input sizes for Simulink
            inputSizes = {[-1, 2]};  % Cell array with variable size specification
        end
        
        function [dt_1, dt_2, dt_3, dt_4, dt_5] = getOutputDataTypeImpl(~)
            dt_1 = 'double';
            dt_2 = 'double';
            dt_3 = 'double';
            dt_4 = 'double';
            dt_5 = 'double';
        end
        
        function [cp_1, cp_2, cp_3, cp_4, cp_5] = isOutputComplexImpl(~)
            cp_1 = false;
            cp_2 = false;
            cp_3 = false;
            cp_4 = false;
            cp_5 = false;
        end
        
        function [fp_1, fp_2, fp_3, fp_4, fp_5] = isOutputFixedSizeImpl(~)
            fp_1 = true;
            fp_2 = true;
            fp_3 = true;
            fp_4 = true;
            fp_5 = true;
        end
        
        function validateInputsImpl(obj, gAlfaDiscNorm)
            % Validate inputs
            validateattributes(gAlfaDiscNorm, {'double'}, {'2d', 'finite'}, 'modSF', 'gAlfaDiscNorm');
            
            % Check that input has exactly 2 columns
            if size(gAlfaDiscNorm, 2) ~= 2
                error('modSF:InvalidInput', ...
                    'gAlfaDiscNorm must have exactly 2 columns [position, angle]. Current size: [%d, %d]', ...
                    size(gAlfaDiscNorm, 1), size(gAlfaDiscNorm, 2));
            end
            
            % Check that we have at least some data points
            if size(gAlfaDiscNorm, 1) < 2
                error('modSF:InvalidInput', ...
                    'gAlfaDiscNorm must have at least 2 rows for interpolation. Current rows: %d', ...
                    size(gAlfaDiscNorm, 1));
            end
            
            % Check position values are in valid range (assuming normalized coordinates)
            pos_values = gAlfaDiscNorm(:, 1);
            if any(pos_values < -1) || any(pos_values > 1)
                warning('modSF:PositionRange', ...
                    'Position values should typically be in range [-1, 1]. Current range: [%.3f, %.3f]', ...
                    min(pos_values), max(pos_values));
            end
        end
    end
    
    methods (Access = private)
        
        function oDPolares(obj)
            archPolar = fullfile(obj.dirPolares, ...
                sprintf('%s_Re%.0f.txt', obj.airfoil, round(obj.Re)));
            
            % Crear polares
            objPolar = polar(obj.airfoil, obj.Re, obj.xfoilPath, archPolar, obj.rAlfa);
            
            % Revisar si existe archivo polar
            if exist(archPolar, 'file')
                disp(['Loading polars: ', archPolar]);
                datPolares = objPolar.leer();
                disp('Polars loaded successfully');
            else
                disp(['Generating polars with XFOIL: ', archPolar]);
                objPolar.generar();
                datPolares = objPolar.leer();
                disp('Polars generated and loaded successfully');
            end
            
            % Extraer datos
            pAlfa = datPolares.alpha * (pi/180);
            pCl = datPolares.Cl;
            
            % Regresión
            [obj.pM, pB] = eRegionLineal(pAlfa, pCl, obj.umbral);
            obj.pAlfa0 = -pB / obj.pM;
        end
        
        function initDom(obj)
            % Inicializar dominio
            
            obj.dPhi = pi / obj.N;
            obj.Phi = zeros(obj.N-1, 1);
            
            for i = 1:(obj.N-1)
                obj.Phi(i) = i * obj.dPhi;
            end
            
            obj.z = 0.5 * obj.b * cos(obj.Phi);
            obj.gAlfaNorm = 2 * obj.z / obj.b;
            obj.gAlfaNorm(:,2) = 0;
        end
        
        function calMatInf(obj)
            % Se ejecuta en la inicialización
            
            obj.Fij = zeros(obj.N-1, obj.N-1);
            
            for j = 1:(obj.N-1)
                for i = 1:(obj.N-1)
                    obj.Fij(i, j) = calcFij(obj.b, obj.pM, obj.c, j, obj.Phi(i));
                end
            end
            
            obj.invFij = inv(obj.Fij);
        end
        
    end
end
