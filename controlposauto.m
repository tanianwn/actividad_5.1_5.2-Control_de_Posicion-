% Limpieza de pantalla
clear all
close all
clc

% 1 TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf=15;           
ts=0.1;            
t=0:ts:tf;         
N= length(t);      

% 2 CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1(1)=0;  
y1(1)=0;  
phi(1)=pi/2; 

% 3 POSICION DESEADA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hxd=10;
hyd=9;

hx(1)= x1(1);       
hy(1)= y1(1);       

% AUTO-TUNING
K_max = 1.2;    % Ganancia máxima para distancias largas
K_min = 0.4;    % Ganancia mínima para precisión al llegar
alpha = 0.5;    % Factor de suavizado

% 4 CONTROL, BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N 
    % a) Errores de control
    hxe(k) = hxd - hx(k);
    hye(k) = hyd - hy(k);
    
    % Magnitud del error de posición actual
    Error(k) = sqrt(hxe(k)^2 + hye(k)^2);
    
    % ---ALGORITMO AUTO-SINTONIZABLE---
    % Calculamos una ganancia escalar que depende del error
    k_adapt = K_min + (K_max - K_min) * (1 - exp(-alpha * Error(k)));
    
    % b) Matriz Jacobiana
    J = [cos(phi(k)) -sin(phi(k));...
         sin(phi(k))  cos(phi(k))];
    
    % c) Matriz de Ganancias Adaptativa
    % Aplicamos la ganancia calculada dinámicamente
    K = [k_adapt 0;...
         0       k_adapt];
    
    % d) Ley de Control
    he = [hxe(k); hye(k)];
    qpRef = pinv(J) * K * he;
    
    v(k) = qpRef(1);   % Velocidad lineal
    w(k) = qpRef(2);   % Velocidad angular
    
    % 5 APLICACIÓN DE CONTROL AL ROBOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    phi(k+1) = phi(k) + w(k) * ts; 
           
    xp1 = v(k) * cos(phi(k)); 
    yp1 = v(k) * sin(phi(k));
 
    x1(k+1) = x1(k) + xp1 * ts; 
    y1(k+1) = y1(k) + yp1 * ts; 
    
    hx(k+1) = x1(k+1); 
    hy(k+1) = y1(k+1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%
scene=figure;  
set(scene,'Color','white','Name','Simulación Robot Diferencial'); 
set(gca,'FontWeight','bold');
sizeScreen=get(0,'ScreenSize'); 
set(scene,'position',sizeScreen); 
camlight('headlight'); 
axis equal; grid on; box on;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view([-0.1 35]); 
axis([-11 11 -11 11 0 1]); 

scale = 4;
try
    MobileRobot_5; 
    H1=MobilePlot_4(x1(1),y1(1),phi(1),scale); hold on;
catch
    disp('Nota: Funciones de dibujo no encontradas, se mostrará solo trayectoria.');
end

H2=plot3(hx(1),hy(1),0,'r','lineWidth',2); hold on;
H3=plot3(hxd,hyd,0,'bo','lineWidth',2); 
H4=plot3(hx(1),hy(1),0,'go','lineWidth',2);

step=2; 
for k=1:step:N
    if exist('H1','var'); delete(H1); end
    if exist('MobilePlot_4','file')
        H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    end
    set(H2, 'XData', hx(1:k), 'YData', hy(1:k), 'ZData', zeros(1,k));
    pause(0.01);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Graficas de Análisis %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph=figure;  
set(graph,'position',sizeScreen,'Color','white','Name','Análisis de Control Adaptativo'); 
subplot(311)
plot(t,v,'b','LineWidth',2),grid on,title('Velocidad Lineal (Adaptativa)'),ylabel('m/s');
subplot(312)
plot(t,w,'g','LineWidth',2),grid on,title('Velocidad Angular'),ylabel('rad/s');
subplot(313)
plot(t,Error,'r','LineWidth',2),grid on,title('Error de Posición'),xlabel('Tiempo [s]'),ylabel('metros');