%% Respuesta al Escalón - Ball and Beam usando impulse() de MATLAB
% Simulación de 30 segundos - Solo Entrada y Salida

clear all; close all; clc;

%% Parámetros físicos REALES del sistema
m = 0.111;          % Masa de la pelota (kg)
R = 0.015;          % Radio de la pelota (m)
g = 9.8;            % Aceleración gravitacional (m/s²)
L = 0.13;           % Longitud de la viga (13 cm)
d = L/2;            % Distancia al centro
J = 9.99e-6;        % Momento de inercia (kg·m²)

%% Controlador PID
Kp = 0.8;
Ki = 0.2;
Kd = 0.2;

%% Función de transferencia del sistema
s = tf('s');
system_gain = m*g*d/(L*(J/R^2 + m));
P_ball = system_gain/s^2;

% Sistema con controlador PID
G_pid = pid(Kp, Ki, Kd);
T_closed = feedback(G_pid * P_ball, 1);

fprintf('=== RESPUESTA AL ESCALÓN - BALL AND BEAM ===\n');
fprintf('Sistema: G(s) = %.6f/s²\n', system_gain);
fprintf('Controlador PID: Kp=%.1f, Ki=%.1f, Kd=%.1f\n', Kp, Ki, Kd);
fprintf('Simulación: 30 segundos\n\n');

%% Simulación de 30 segundos
t_sim = 0:0.01:30;  % 30 segundos con resolución de 10ms

%% MÉTODO 1: Usar impulse() para obtener respuesta al escalón
% La respuesta al escalón es la integral de la respuesta al impulso
[h_impulse, t_impulse] = impulse(T_closed, t_sim);

% Integrar numéricamente para obtener respuesta al escalón
dt = t_impulse(2) - t_impulse(1);
y_step = cumsum(h_impulse) * dt;

%% Definir entrada escalón y salida del sistema
% ENTRADA: Escalón aplicado en t = 2 segundos
entrada_escalon = zeros(size(t_impulse));
t_escalon = 2;  % Aplicar escalón a los 2 segundos
escalon_indices = t_impulse >= t_escalon;
entrada_escalon(escalon_indices) = 3.5;  % Escalón de 3.5 cm

% SALIDA: Respuesta del sistema al escalón
% Posición inicial: 6.5 cm (centro), destino: 6.5 + 3.5 = 10 cm
salida_sistema = 6.5 * ones(size(t_impulse));  % Posición inicial
for i = 1:length(t_impulse)
    if t_impulse(i) >= t_escalon
        tiempo_desde_escalon = i - find(escalon_indices, 1) + 1;
        if tiempo_desde_escalon <= length(y_step)
            salida_sistema(i) = 6.5 + y_step(tiempo_desde_escalon) * 3.5;
        else
            salida_sistema(i) = 6.5 + y_step(end) * 3.5;
        end
    end
end

%% Crear gráfica principal: ENTRADA vs SALIDA
figure('Position', [100, 100, 1400, 800]);

%% Gráfico 1: ENTRADA y SALIDA en el mismo plot
subplot(2,1,1);
plot(t_impulse, entrada_escalon, 'r-', 'LineWidth', 4);
hold on;
plot(t_impulse, salida_sistema, 'b-', 'LineWidth', 3);

% Líneas de referencia
xline(t_escalon, 'g--', 'Escalón aplicado', 'LineWidth', 2, 'FontSize', 12);
yline(6.5, 'k:', 'Posición inicial', 'LineWidth', 1.5);
yline(10, 'k:', 'Posición final deseada', 'LineWidth', 1.5);

% Límites de la viga
yline(0, 'k-', 'Límite viga (0 cm)', 'LineWidth', 2);
yline(13, 'k-', 'Límite viga (13 cm)', 'LineWidth', 2);

title('RESPUESTA AL ESCALÓN - Sistema Ball and Beam (30 segundos)', 'FontSize', 16, 'FontWeight', 'bold');
xlabel('Tiempo (s)', 'FontSize', 14);
ylabel('Posición (cm)', 'FontSize', 14);
legend('ENTRADA (Escalón de referencia)', 'SALIDA (Posición de la pelota)', ...
       'Momento del escalón', 'Location', 'best', 'FontSize', 12);
grid on;
xlim([0, 30]);
ylim([-1, 14]);

% Anotaciones importantes
text(15, 11, sprintf('Escalón: %.1f cm\nEn t = %.0f s', 3.5, t_escalon), ...
     'FontSize', 12, 'BackgroundColor', 'yellow', 'EdgeColor', 'black');
text(25, 2, sprintf('Viga: %.0f cm', L*100), 'FontSize', 12, 'BackgroundColor', 'white');

%% Gráfico 2: Análisis detallado de la respuesta
subplot(2,1,2);

% Error del sistema
error_sistema = (6.5 + entrada_escalon) - salida_sistema;
plot(t_impulse, error_sistema, 'r-', 'LineWidth', 2);
hold on;

% Velocidad de la pelota (derivada de la posición)
velocidad = gradient(salida_sistema, dt);
plot(t_impulse, velocidad, 'g-', 'LineWidth', 2);

% Línea de cero
plot([0 30], [0 0], 'k--', 'LineWidth', 1);

title('Análisis de la Respuesta', 'FontSize', 14);
xlabel('Tiempo (s)', 'FontSize', 12);
ylabel('Error (cm) / Velocidad (cm/s)', 'FontSize', 12);
legend('Error del sistema', 'Velocidad de la pelota', 'Location', 'best');
grid on;
xlim([0, 30]);

%% Análisis numérico de la respuesta
fprintf('=== ANÁLISIS DE LA RESPUESTA AL ESCALÓN ===\n');

% Encontrar parámetros de la respuesta
if isstable(T_closed)
    % Tiempo de establecimiento (2% del valor final)
    valor_final = salida_sistema(end);
    banda_2_porciento = 0.02 * 3.5;  % 2% del escalón
    
    indices_establecido = find(abs(salida_sistema(escalon_indices) - valor_final) <= banda_2_porciento);
    if ~isempty(indices_establecido)
        t_establecimiento = t_impulse(find(escalon_indices, 1) + indices_establecido(1) - 1) - t_escalon;
        fprintf('• Tiempo de establecimiento (2%%): %.2f segundos\n', t_establecimiento);
    else
        fprintf('• Sistema aún se está estabilizando después de 30s\n');
    end
    
    % Sobreoscilación
    max_valor = max(salida_sistema(escalon_indices));
    sobrepaso = ((max_valor - valor_final) / 3.5) * 100;
    fprintf('• Sobreoscilación: %.1f%%\n', sobrepaso);
    
    % Error en estado estacionario
    error_final = abs((6.5 + 3.5) - valor_final);
    fprintf('• Error en estado estacionario: %.3f cm\n', error_final);
    
else
    fprintf('• SISTEMA INESTABLE - Requiere ajuste de controlador\n');
end

fprintf('• Posición inicial: 6.5 cm (centro de la viga)\n');
fprintf('• Escalón aplicado: +3.5 cm en t = %.0f s\n', t_escalon);
fprintf('• Posición final deseada: 10.0 cm\n');
fprintf('• Posición final alcanzada: %.2f cm\n', salida_sistema(end));

%% Información del método usado
fprintf('\n=== MÉTODO UTILIZADO ===\n');
fprintf('• Se usó la función impulse() de MATLAB\n');
fprintf('• Se integró la respuesta al impulso para obtener respuesta al escalón\n');
fprintf('• Simulación de 30 segundos con muestreo de 10 ms\n');
fprintf('• Sistema: Ball and Beam real con parámetros físicos\n');

%% Guardar resultados
print('respuesta_escalon_30s_ball_beam', '-dpng', '-r300');
save('escalon_30s_data.mat', 't_impulse', 'entrada_escalon', 'salida_sistema', ...
     'T_closed', 'system_gain', 'error_sistema');

fprintf('\n=== ARCHIVOS GUARDADOS ===\n');
fprintf('• Gráfica: respuesta_escalon_30s_ball_beam.png\n');
fprintf('• Datos: escalon_30s_data.mat\n');

fprintf('\n=== RESPUESTA AL ESCALÓN COMPLETADA ===\n');
fprintf('Entrada: Escalón de %.1f cm aplicado en t = %.0f s\n', 3.5, t_escalon);
fprintf('Salida: Posición de la pelota en la viga de %.0f cm\n', L*100);
fprintf('Método: impulse() de MATLAB para obtener respuesta al escalón\n');