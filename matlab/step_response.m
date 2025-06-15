% Análisis de respuesta al escalón
% Leer datos del serial del Arduino

% Cargar datos (ejemplo)
data = readmatrix('step_response_data.csv');
time = data(:,1) / 1000; % Convertir a segundos
setpoint = data(:,2);
output = data(:,3);

% Graficar respuesta
figure(1);
plot(time, setpoint, 'r--', 'LineWidth', 2);
hold on;
plot(time, output, 'b-', 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('Posición (cm)');
title('Respuesta al Escalón del Sistema');
legend('Setpoint', 'Salida del Sistema');
grid on;

% Identificar parámetros del sistema
% Encontrar valores característicos
finalValue = mean(output(end-50:end));
initialValue = output(1);
stepHeight = finalValue - initialValue;

% Tiempo de subida (10% a 90%)
idx10 = find(output >= initialValue + 0.1*stepHeight, 1);
idx90 = find(output >= initialValue + 0.9*stepHeight, 1);
riseTime = time(idx90) - time(idx10);

% Tiempo de establecimiento (±2%)
tolerance = 0.02 * stepHeight;
settlingIdx = find(abs(output - finalValue) <= tolerance, 1);
settlingTime = time(settlingIdx);

% Sobrepaso máximo
maxValue = max(output);
overshoot = ((maxValue - finalValue) / stepHeight) * 100;

% Mostrar parámetros
fprintf('Parámetros del sistema:\n');
fprintf('Tiempo de subida: %.2f s\n', riseTime);
fprintf('Tiempo de establecimiento: %.2f s\n', settlingTime);
fprintf('Sobrepaso: %.1f%%\n', overshoot);

% Identificación de función de transferencia
% Aproximación de primer orden: G(s) = K/(τs + 1)
tau = settlingTime / 4; % Aproximación
K = stepHeight;
s = tf('s');
G_approx = K / (tau*s + 1);

% Comparar respuestas
[y_model, t_model] = step(G_approx, time(end));
figure(2);
plot(time, output, 'b-', t_model, y_model + initialValue, 'r--');
xlabel('Tiempo (s)');
ylabel('Posición (cm)');
title('Comparación: Sistema Real vs Modelo');
legend('Sistema Real', 'Modelo Identificado');
grid on;

% Diagrama de polos y ceros
figure(3);
pzmap(G_approx);
title('Diagrama Polos-Ceros del Modelo');