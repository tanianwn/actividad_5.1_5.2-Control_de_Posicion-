clear
close all
clc

ts = 0.1;            % Tiempo de muestreo en segundos (s)

puntos= [ 0 5 2.5 0 2.5 7.5 12.5 12.5 15 17.5 17.5 15 12.5 15 12.5 15 17.5 15 17.5 22.5 27.5 30 27.5 25 30 30 25 22.5 17.5 15 12.5 7.5 2.5 0;
          2.5 5 7.5 12.5 17.5 17.5 15 2.5 0 2.5 15 17.5 15 17.5 20 17.5 20 17.5 15 17.5 17.5 12.5 7.5 5 2.5 0 -2.5 0 2.5 0 2.5 0 -2.5 0];
%Velocidades
v_avanzar = 1;       % Velocidad lineal constante al avanzar (m/s)
w_rotar = pi/2;      % Velocidad angular constante al rotar (rad/s)

u = []; % Se inicializan vacíos y se irán llenando de acuerdo a los puntos
w = [];

%Variables temporales para el calculo
x_calc = 0; 
y_calc = 0; 
phi_calc = pi/2; 

% Calculamos u y w tramo por tramo
for i = 2:size(puntos,2)
    x_obj = puntos(1,i);
    y_obj = puntos(2,i);
    
    %Calculo de distancia y angulo
    distancia = sqrt((x_obj - x_calc)^2 + (y_obj - y_calc)^2);
    angulo_deseado = atan2(y_obj - y_calc, x_obj - x_calc);
    
    %Diferencia de ángulo para saber cuánto rotar
    delta_phi = angulo_deseado - phi_calc;
    delta_phi = atan2(sin(delta_phi), cos(delta_phi)); 
    
    %Primero rota
    tiempo_rot = abs(delta_phi) / w_rotar;
    muestras_rot = round(tiempo_rot / ts);
    if muestras_rot > 0
        w = [w, sign(delta_phi) * w_rotar * ones(1, muestras_rot)];
        u = [u, zeros(1, muestras_rot)];
    end
    phi_calc = phi_calc + delta_phi;
    
    %Despues avanza
    tiempo_av = distancia / v_avanzar;
    muestras_av = round(tiempo_av / ts);
    if muestras_av > 0
        u = [u, v_avanzar * ones(1, muestras_av)];
        w = [w, zeros(1, muestras_av)];
    end

    x_calc = x_obj;
    y_calc = y_obj;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ahora calculamos tf y N dependiendo de qué tan largos quedaron 'u' y 'w'
N = length(u);       % Muestras totales de la trayectoria generada
tf = (N-1)*ts;       % Tiempo de simulacion en segundos (s)
t = 0: ts: tf;       % Vector de tiempo

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = zeros (1,N+1);  % Posición en el centro del eje que une las ruedas (eje x) en metros (m)
y1 = zeros (1,N+1);  % Posición en el centro del eje que une las ruedas (eje y) en metros (m)
phi = zeros(1, N+1); % Orientacion del robot en radianes (rad)
x1(1) = 0;    % Posicion inicial eje x
y1(1) = 0;   % Posicion inicial eje y
phi(1) = pi/2;   % Orientacion inicial del robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTO DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%
hx = zeros(1, N+1);  % Posicion en el punto de control (eje x) en metros (m)
hy = zeros(1, N+1);  % Posicion en el punto de control (eje y) en metros (m)
hx(1) = x1(1); % Posicion en el punto de control del robot en el eje x
hy(1) = y1(1); % Posicion en el punto de control del robot en el eje y

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N 
    
    phi(k+1)=phi(k)+w(k)*ts; % Integral numérica (método de Euler)
    
    %%%%%%%%%%%%%%%%%%%%% MODELO CINEMATICO %%%%%%%%%%%%%%%%%%%%%%%%%
    %Aplicamos el modelo cinemático diferencial para obtener las
    %velocidades en x, y, phi
    xp1=u(k)*cos(phi(k+1)); 
    yp1=u(k)*sin(phi(k+1));
    phip= w(k);
    x1(k+1)=x1(k) + xp1*ts ; % Integral numérica (método de Euler)
    y1(k+1)=y1(k) + yp1*ts ; % Integral numérica (método de Euler)
    
    
    % Posicion del robot con respecto al punto de control
    hx(k+1)=x1(k+1); 
    hy(k+1)=y1(k+1);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% a) Configuracion de escena
scene=figure;  % Crear figura (Escena)
set(scene,'Color','white'); % Color del fondo de la escena
set(gca,'FontWeight','bold') ;% Negrilla en los ejes y etiquetas
sizeScreen=get(0,'ScreenSize'); % Retorna el tamaño de la pantalla del computador
set(scene,'position',sizeScreen); % Congigurar tamaño de la figura
camlight('headlight'); % Luz para la escena
axis equal; % Establece la relación de aspecto para que las unidades de datos sean las mismas en todas las direcciones.
grid on; % Mostrar líneas de cuadrícula en los ejes
box on; % Mostrar contorno de ejes
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Etiqueta de los eje
view([-0.1 35]); % Orientacion de la figura
axis([-10 40 -15 30 0 1]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]
% b) Graficar robots en la posicion inicial
scale = 4;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;
% c) Graficar Trayectorias
H2=plot3(hx(1),hy(1),0,'r','lineWidth',2);
% d) Bucle de simulacion de movimiento del robot
step=1; % pasos para simulacion
for k=1:step:N
    delete(H1);    
    delete(H2);
    
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
    
    pause(ts);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;  % Crear figura (Escena)
set(graph,'position',sizeScreen); % Congigurar tamaño de la figura
subplot(211)
plot(t,u,'b','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('m/s'),legend('u');
subplot(212)
plot(t,w,'r','LineWidth',2),grid('on'),xlabel('Tiempo [s]'),ylabel('[rad/s]'),legend('w');
