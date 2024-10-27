function varargout = carro(varargin)
% CARRO MATLAB code for carro.fig
%      CARRO, by itself, creates a new CARRO or raises the existing
%      singleton*.
%
%      H = CARRO returns the handle to a new CARRO or the handle to
%      the existing singleton*.
%
%      CARRO('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CARRO.M with the given input arguments.
%
%      CARRO('Property','Value',...) creates a new CARRO or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before carro_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to carro_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".

% Edit the above text to modify the response to help carro

% Last Modified by GUIDE v2.5 27-Sep-2024 16:40:16

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @carro_OpeningFcn, ...
                   'gui_OutputFcn',  @carro_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before carro is made visible.
function carro_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to carro (see VARARGIN)

% Choose default command line output for carro
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes carro wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Configure serial communication when GUI opens
global s;
s = serialport("COM4", 9600); % Reemplaza "COM4" por el puerto adecuado en tu sistema

% Variable global para indicar si el cronómetro ha sido iniciado
global cronometroIniciado tiempoTranscurrido;
cronometroIniciado = 0;  % Inicialmente, el cronómetro no ha sido iniciado
tiempoTranscurrido = 0;  % Variable para contar los segundos

% Crear un Timer que se ejecute cada segundo para el cronómetro
handles.timerCronometro = timer('ExecutionMode', 'fixedRate', 'Period', 1, ...
                      'TimerFcn', @(~,~) actualizarCronometro(hObject, handles));

% Crear un Timer para leer datos de la UART
handles.timerUART = timer('ExecutionMode', 'fixedRate', 'Period', 0.1, ...
                          'TimerFcn', @(~,~) leerDatosUART(hObject, handles));
    
% Iniciar el Timer para leer datos de la UART
start(handles.timerUART);
                      
                  
% Guarda el timerCronometro en handles
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = carro_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Ejecutar acción al presionar el botón Adelante
function adelante_Callback(hObject, eventdata, handles)
   global s cronometroIniciado;

    % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        write(s, 'C', 'char');  % Enviar comando 'C' para iniciar el cronómetro en el STM32
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
    end
    write(s, 'W', 'char');  % Enviar comando 'W' por UART para mover hacia adelante
    % Cambiar el color del botón a verde
    set(hObject, 'BackgroundColor', [0, 1, 0]);
    
    % Cambiar el texto del static text con tag=text5
    set(handles.text5, 'String', 'Moviendo Adelante');
    
    % Restaurar el color original después de un tiempo
    pause(3);  % Pausa breve
    set(hObject, 'BackgroundColor', get(0, 'defaultUicontrolBackgroundColor'));


% --- Ejecutar acción al presionar el botón Atras
function atras_Callback(hObject, eventdata, handles)
    global s cronometroIniciado;

    % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        write(s, 'C', 'char');  % Enviar comando 'C' para iniciar el cronómetro en el STM32
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
    end
    write(s, 'S', 'char');  % Enviar comando 'S' por UART para mover hacia atrás
    % Cambiar el color del botón a verde
    set(hObject, 'BackgroundColor', [0, 1, 0]);
    
    % Cambiar el texto del static text con tag=text5
    set(handles.text5, 'String', 'Moviendo Atrás');
    
    % Restaurar el color original después de un tiempo
    pause(3);  % Pausa breve
    set(hObject, 'BackgroundColor', get(0, 'defaultUicontrolBackgroundColor'));


% --- Ejecutar acción al presionar el botón Izquierda
function izquierda_Callback(hObject, eventdata, handles)
    global s cronometroIniciado;

    % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        write(s, 'C', 'char');  % Enviar comando 'C' para iniciar el cronómetro en el STM32
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
    end
    write(s, 'A', 'char');  % Enviar comando 'A' por UART para mover hacia la izquierda
    % Cambiar el color del botón a verde
    set(hObject, 'BackgroundColor', [0, 1, 0]);
    
    % Cambiar el texto del static text con tag=text5
    set(handles.text5, 'String', 'Girando Izquierda');
    
    % Restaurar el color original después de un tiempo
    pause(3);  % Pausa breve
    set(hObject, 'BackgroundColor', get(0, 'defaultUicontrolBackgroundColor'));


% --- Ejecutar acción al presionar el botón Derecha
function derecha_Callback(hObject, eventdata, handles)
    global s cronometroIniciado;

    % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        write(s, 'C', 'char');  % Enviar comando 'C' para iniciar el cronómetro en el STM32
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
    end
    write(s, 'D', 'char');  % Enviar comando 'D' por UART para mover hacia la derecha
    % Cambiar el color del botón a verde
    set(hObject, 'BackgroundColor', [0, 1, 0]);
    
    % Cambiar el texto del static text con tag=text5
    set(handles.text5, 'String', 'Girando Derecha');
    
    % Restaurar el color original después de un tiempo
    pause(3);  % Pausa breve
    set(hObject, 'BackgroundColor', get(0, 'defaultUicontrolBackgroundColor'));


% --- Ejecutar acción al presionar el botón Detener
function detener_Callback(hObject, eventdata, handles)
    global s;
    write(s, 'X', 'char');  % Enviar comando 'X' por UART para detener el robot
    % Cambiar el texto del static text con tag=text5
    set(handles.text5, 'String', 'parqueado');

% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
    global s tiempoTranscurrido cronometroIniciado;
    write(s, 'R', 'char');
    % Reiniciar el cronómetro en la GUI
    tiempoTranscurrido = 0;  % Reiniciar la variable de tiempo transcurrido
    set(handles.text1, 'String', '00:00');  % Actualizar el cronómetro en la interfaz a "00:00"

    % Detener el timer del cronómetro en la GUI
    stop(handles.timerCronometro);  % Detener el Timer del cronómetro

    % Marcar que el cronómetro no ha sido iniciado
    cronometroIniciado = 0;
    
    % Guarda los cambios en handles
    guidata(hObject, handles);
    
% --- Función para actualizar el cronómetro en el GUI
function actualizarCronometro(hObject, handles)
    global tiempoTranscurrido;

    % Incrementar el tiempo transcurrido
    tiempoTranscurrido = tiempoTranscurrido + 1;
    
    % Convertir el tiempo en minutos y segundos
    minutos = floor(tiempoTranscurrido / 60);
    segundos = rem(tiempoTranscurrido, 60);
    
    % Formatear el tiempo como "mm:ss"
    cronometroStr = sprintf('%02d:%02d', minutos, segundos);
    
    % Actualizar el 'static text' en el GUI con el nuevo valor del cronómetro
    set(handles.text1, 'String', cronometroStr);

    % Guarda los cambios en handles
    guidata(hObject, handles);
    
function leerDatosUART(hObject, handles)
    global s;
    
    % Leer los datos disponibles en el buffer UART
    if s.NumBytesAvailable > 0
        data = read(s, s.NumBytesAvailable, "char");  % Leer los bytes disponibles
       
        % Comparar el mensaje recibido para ver si se detectó algo o no
        if contains(data, "infrarojoSi")
            % Si se detecta algo, actualizar el text2 para indicar la detección
            set(handles.text2, 'String', 'Detección: Sí');
        elseif contains(data, "infrarojoNo")
            % Si no se detecta nada, actualizar el text2 para indicar que no hay detección
            set(handles.text2, 'String', 'Detección: No');
        end
        %consulta por el estado del cny
        if contains(data, "cnySi")
            % Si se detecta algo, actualizar el text2 para indicar la detección
            set(handles.text3, 'String', 'Detección infr: Sí');
        elseif contains(data, "cnyNo")
            % Si no se detecta nada, actualizar el text2 para indicar que no hay detección
            set(handles.text3, 'String', 'Detección cny: No');    
        end
    end

    % Guardar cambios en handles
    guidata(hObject, handles);
    
    
    


