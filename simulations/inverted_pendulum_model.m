% Modelado y Simulación de Péndulo Invertido
% Autor: Agustín Ezequiel Feroci

clear; clc;
disp('Cargando parámetros del sistema dinámico...');

M = 0.5;  % Masa del carro (kg)
m = 0.2;  % Masa del péndulo (kg)
b = 0.1;  % Coeficiente de fricción del carro
i = 0.006;% Inercia del péndulo
g = 9.81; % Gravedad (m/s^2)
l = 0.3;  % Longitud al centro de masa (m)

p = i*(M+m) + M*m*l^2;

A = [0, 1, 0, 0;
     0, -(i+m*l^2)*b/p, (m^2*g*l^2)/p, 0;
     0, 0, 0, 1;
     0, -m*l*b/p, m*g*l*(M+m)/p, 0];

B = [0; (i+m*l^2)/p; 0; m*l/p];
C = [1, 0, 0, 0; 0, 0, 1, 0];
D = [0; 0];

sys = ss(A, B, C, D);
disp('Sistema en espacio de estados linealizado con éxito.');
