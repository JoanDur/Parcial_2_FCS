%% PARCIAL II - GENERADOR AUTOMÁTICO DE MODELOS, SIMULACIÓN Y ANIMACIÓN
%% SISTEMA 5: MICRÓFONO DE CONDENSADOR
%% UNIVERSIDAD DISTRITAL - 2026

clear; clc; close all;

%% 1. DEFINICIÓN DE PARÁMETROS Y CÁLCULOS
fprintf('1. Calculando parámetros del Sistema 5...\n');
M = 1e-4;         % Masa del diafragma (kg)
K = 1000;         % Constante del resorte (N/m)
B = 15;           % Amortiguamiento (N*s/m)
R = 10e6;         % Resistencia (Ohms)
L = 0.01;         % Inductancia (H)

K_v = 2.21e-10;   % Ganancia Voltaje -> Carga
K_cx = 4.42e-6;   % Ganancia Desplazamiento -> Carga
K_q = 2.26e7;     % Ganancia Carga -> Fuerza
K_x = 500;        % Rigidez negativa (N/m)
K_neta = K - K_x; 

% Función de transferencia
s = tf('s');
G_M = 1 / (M*s^2 + B*s + K_neta);
H_E = (K_cx * K_q) / (K_v*L*s^2 + K_v*R*s + 1);
G_total = G_M / (1 + G_M * H_E);

% Espacio de Estados
sys_ss = ss(G_total);
[A, B_mat, C, D] = ssdata(sys_ss);

% Extraer numerador y denominador como vectores para Simulink
[num, den] = tfdata(G_total, 'v');

%% 2. CREACIÓN AUTOMÁTICA DEL MODELO SIMULINK
fprintf('2. Construyendo el modelo Simulink por código...\n');
nombre_modelo = 'Validacion_Sistema5';

% Cerrar el modelo si ya estaba abierto para evitar errores
if bdIsLoaded(nombre_modelo)
    close_system(nombre_modelo, 0);
end

% Crear y abrir un nuevo modelo de Simulink
new_system(nombre_modelo);
open_system(nombre_modelo);

% --- AGREGAR BLOQUES AL MODELO ---
add_block('simulink/Sources/Step', [nombre_modelo, '/Fuerza_Acustica']);
set_param([nombre_modelo, '/Fuerza_Acustica'], 'Time', '0', 'Before', '0', 'After', '0.001');

add_block('simulink/Continuous/Transfer Fcn', [nombre_modelo, '/FT_Total']);
set_param([nombre_modelo, '/FT_Total'], 'Numerator', mat2str(num), 'Denominator', mat2str(den));

add_block('simulink/Continuous/State-Space', [nombre_modelo, '/Forma_Canonica']);
set_param([nombre_modelo, '/Forma_Canonica'], 'A', 'A', 'B', 'B_mat', 'C', 'C', 'D', 'D');

add_block('simulink/Signal Routing/Mux', [nombre_modelo, '/Mux_Comparador']);
set_param([nombre_modelo, '/Mux_Comparador'], 'Inputs', '2'); 

add_block('simulink/Sinks/Scope', [nombre_modelo, '/Scope_Validacion']);

% --- POSICIONAR LOS BLOQUES ---
set_param([nombre_modelo, '/Fuerza_Acustica'], 'Position', [50, 100, 80, 130]);
set_param([nombre_modelo, '/FT_Total'], 'Position', [200, 50, 350, 100]);
set_param([nombre_modelo, '/Forma_Canonica'], 'Position', [200, 150, 350, 200]);
set_param([nombre_modelo, '/Mux_Comparador'], 'Position', [450, 100, 455, 150]);
set_param([nombre_modelo, '/Scope_Validacion'], 'Position', [520, 108, 550, 142]);

% --- CONECTAR LOS BLOQUES ---
add_line(nombre_modelo, 'Fuerza_Acustica/1', 'FT_Total/1', 'autorouting', 'on');
add_line(nombre_modelo, 'Fuerza_Acustica/1', 'Forma_Canonica/1', 'autorouting', 'on');
add_line(nombre_modelo, 'FT_Total/1', 'Mux_Comparador/1', 'autorouting', 'on');
add_line(nombre_modelo, 'Forma_Canonica/1', 'Mux_Comparador/2', 'autorouting', 'on');
add_line(nombre_modelo, 'Mux_Comparador/1', 'Scope_Validacion/1', 'autorouting', 'on');

%% 3. CONFIGURACIÓN Y GUARDADO (CON CORRECCIÓN)
fprintf('3. Guardando y exportando modelo a versión R2019b...\n');
set_param(nombre_modelo, 'StopTime', '0.05');

% PRIMERO: Guardar el modelo en la versión actual en la carpeta de trabajo
save_system(nombre_modelo, [nombre_modelo '.slx']);

% SEGUNDO: Exportar el modelo guardado a versión 2019b
save_system(nombre_modelo, 'Modelo_Parcial_S5_2019b.slx', 'ExportToVersion', 'R2019b');
fprintf('¡Éxito! El archivo Modelo_Parcial_S5_2019b.slx ha sido generado y guardado.\n');

%% 4. CORRER SIMULACIÓN Y MOSTRAR SCOPE
fprintf('4. Ejecutando simulación de Simulink...\n');
simOut = sim(nombre_modelo);
open_system([nombre_modelo, '/Scope_Validacion']);

%% 5. SIMULACIÓN ANALÍTICA Y ANIMACIÓN DEL DIAFRAGMA
fprintf('5. Iniciando animación interactiva del diafragma...\n');
t = linspace(0, 0.05, 1000); % Vector de tiempo (0 a 50 ms)
f_s = 0.001 * ones(size(t)); % Entrada escalón

% Generar los datos matemáticos puros usando lsim
[x_tf, t_out] = lsim(G_total, f_s, t);

figure('Name', 'Animación Física: Diafragma del Micrófono', 'NumberTitle', 'off');
escala_visual = 2e4; % Escalado para hacer visible el movimiento micrométrico

for i = 1:10:length(t_out) % Salta de 10 en 10 para que la animación sea fluida
    clf; % Limpiar figura actual
    
    % Dibujar Placa Fija (Derecha)
    plot([0.5, 0.5], [-0.5, 0.5], 'k-', 'LineWidth', 5); hold on;
    text(0.52, 0.4, 'Placa Fija (A)', 'FontWeight', 'bold');
    
    % Calcular posición dinámica de la placa móvil mediante traslación simple
    pos_x_movil = -0.5 + (x_tf(i) * escala_visual);
    
    % Dibujar Placa Móvil / Diafragma (Izquierda)
    plot([pos_x_movil, pos_x_movil], [-0.4, 0.4], 'r-', 'LineWidth', 4);
    text(pos_x_movil - 0.4, 0.3, 'Diafragma (M)', 'Color', 'r', 'FontWeight', 'bold');
    
    % Dibujar el efecto del Resorte Mecánico
    eje_x_resorte = linspace(-0.9, pos_x_movil, 10);
    eje_y_resorte = 0.1 * sin(2 * pi * (1:10));
    plot(eje_x_resorte, eje_y_resorte, 'b-', 'LineWidth', 1.5);
    plot([-1, -0.9], [0, 0], 'b-', 'LineWidth', 1.5); % Soporte izquierdo
    
    % Configuración de la Ventana Gráfica
    xlim([-1.2, 0.8]);
    ylim([-0.8, 0.8]);
    title(sprintf('Simulación del Micrófono | Tiempo: %.2f ms', t_out(i)*1000), 'FontSize', 12);
    xlabel('Desplazamiento Normalizado Amplificado', 'FontWeight', 'bold');
    grid on;
    
    % Forzar dibujado inmediato
    drawnow;
end
fprintf('Proceso completado exitosamente.\n');