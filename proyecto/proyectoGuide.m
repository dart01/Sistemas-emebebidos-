function varargout = proyectoGuide(varargin)
% PROYECTOGUIDE MATLAB code for proyectoGuide.fig
%      PROYECTOGUIDE, by itself, creates a new PROYECTOGUIDE or raises the existing
%      singleton*.
%
%      H = PROYECTOGUIDE returns the handle to a new PROYECTOGUIDE or the handle to
%      the existing singleton*.
%
%      PROYECTOGUIDE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PROYECTOGUIDE.M with the given input arguments.
%
%      PROYECTOGUIDE('Property','Value',...) creates a new PROYECTOGUIDE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before proyectoGuide_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to proyectoGuide_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help proyectoGuide

% Last Modified by GUIDE v2.5 06-Nov-2024 05:00:56

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @proyectoGuide_OpeningFcn, ...
                   'gui_OutputFcn',  @proyectoGuide_OutputFcn, ...
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


% --- Executes just before proyectoGuide is made visible.
function proyectoGuide_OpeningFcn(hObject, eventdata, handles, varargin)
% Configuración inicial de la GUI
    handles.output = hObject;
    guidata(hObject, handles);
    
    % Configuración del puerto serial
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
function varargout = proyectoGuide_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% Obtener el valor del slider (0 a 180 grados)
    
    global s cronometroIniciado;
     
    
     % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 254], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
    angle = get(hObject, 'Value');
    % Redondear el valor del ángulo
    angle = uint8(round(angle));
    
    % Enviar encabezado y valor del ángulo
    write(s, [1, angle], 'uint8');  % `1` indica slider 1
    % Mostrar el ángulo en el cuadro de texto para retroalimentación
    set(handles.edit1, 'String', angle);
set(handles.text4, 'String', angle);

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% Obtener el valor del slider (0 a 180 grados)
    angle = get(hObject, 'Value');
    global s cronometroIniciado;
    % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 254], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
    % Redondear el valor del ángulo
    angle = uint8(round(angle));  
    % Enviar encabezado y valor del ángulo
    write(s, [2, angle], 'uint8');  % `2` indica slider 2
    
set(handles.edit2, 'String', angle);
set(handles.text5, 'String', angle);

% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% Obtener el valor del slider (0 a 180 grados)
    angle = get(hObject, 'Value');
    global s cronometroIniciado;
     
    
     % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 666], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
    % Redondear el valor del ángulo
    angle = uint8(round(angle));  
     write(s, [3, angle], 'uint8');  % `3` indica slider 3
set(handles.edit3, 'String', angle);
set(handles.text5, 'String', angle);

% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
global s cronometroIniciado;
     
    
     % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 666], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
write(s, [4, 180], 'uint8');  % `4` indica boton=4
set(handles.edit4, 'String', "180");
set(handles.text17, 'String', "open");

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
global s cronometroIniciado;
     
    
     % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 666], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
write(s, [4, 0], 'uint8');  % `5` indica botton=5
set(handles.edit4, 'String', "0");
set(handles.text17, 'String', "close");


function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_izq.
function pushbutton_izq_Callback(hObject, eventdata, handles)
global s cronometroIniciado;
     
    
     % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 666], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
write(s, [7, 100], 'uint8');  % `5` indica botton=5
% Cambiar el texto del static text con tag=text
set(handles.text18, 'String', 'gitando izquierda');

% --- Executes on button press in pushbutton_der.
function pushbutton_der_Callback(hObject, eventdata, handles)
global s cronometroIniciado;
     
    
     % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 666], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
write(s, [6, 100], 'uint8');  % `5` indica botton=5
set(handles.text18, 'String', 'girando derecha');

% --- Executes on button press in pushbutton_reversa.
function pushbutton_reversa_Callback(hObject, eventdata, handles)
global s cronometroIniciado;
     
    
     % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 666], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
write(s, [8, 100], 'uint8');  % `8` indica botton=8
set(handles.text18, 'String', 'reversa');
set(handles.text8, 'String', '50%');
set(handles.text9, 'String', '50%');

% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
global s cronometroIniciado;
     
    
     % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 666], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
write(s, [10, 100], 'uint8');  % `5` indica botton=5
set(handles.text18, 'String', 'detenido');
set(handles.text8, 'String', '0%');
set(handles.text9, 'String', '0%');

% --- Executes on button press in pushbutton_adelante.
function pushbutton_adelante_Callback(hObject, eventdata, handles)
global s cronometroIniciado;
     
    
     % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 666], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
write(s, [5, 100], 'uint8');  % `5` indica botton=5
set(handles.text18, 'String', 'Moviendo Adelante');
set(handles.text8, 'String', '100%');
set(handles.text9, 'String', '100%');

% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
global s cronometroIniciado;
     
    
     % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 666], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
write(s, [11, 2], 'uint8');  % `5` indica botton=5
set(handles.text18, 'String', 'Moviendo Adelante');
set(handles.text8, 'String', '35%');


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
global s cronometroIniciado;
     
    
     % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 666], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
write(s, [12, 2], 'uint8');  % `5` indica botton=5
set(handles.text18, 'String', 'Moviendo Adelante');
set(handles.text8, 'String', '55%');

% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
global s cronometroIniciado;
     
    
     % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 666], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
write(s, [13, 2], 'uint8');  % `5` indica botton=5
set(handles.text18, 'String', 'Moviendo Adelante');
set(handles.text8, 'String', '70%');

% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
global s cronometroIniciado;
     
    
     % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 666], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
write(s, [14, 2], 'uint8');  % `5` indica botton=5
set(handles.text18, 'String', 'Moviendo Adelante');
set(handles.text8, 'String', '90%');

% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
global s cronometroIniciado;
     
    
     % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 666], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
write(s, [15, 1], 'uint8');  % `5` indica botton=5
set(handles.text18, 'String', 'Moviendo Adelante');
set(handles.text9, 'String', '35%');

% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
global s cronometroIniciado;
     
    
     % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 666], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
write(s, [16, 1], 'uint8');  % `5` indica botton=5
set(handles.text18, 'String', 'Moviendo Adelante');
set(handles.text9, 'String', '55%');

% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(hObject, eventdata, handles)
global s cronometroIniciado;
     
    
     % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 666], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
write(s, [17, 1], 'uint8');  % `5` indica botton=5
set(handles.text18, 'String', 'Moviendo Adelante');
set(handles.text9, 'String', '70%');

% --- Executes on button press in pushbutton19.
function pushbutton19_Callback(hObject, eventdata, handles)
global s cronometroIniciado;
     
    
     % Verificar si el cronómetro ya ha sido iniciado, en caso contrario enviar comando para iniciarlo
    if cronometroIniciado == 0
        start(handles.timerCronometro);  % Iniciar el Timer del cronómetro
        cronometroIniciado = 1;  % Marcar que el cronómetro ya ha sido iniciado
        write(s, [9, 666], 'uint8');  % `9` indica iniciar cronometro lcd 666
    end
write(s, [18, 1], 'uint8');  % `5` indica botton=5
set(handles.text18, 'String', 'Moviendo Adelante');
set(handles.text9, 'String', '90%');

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
    set(handles.text2, 'String', cronometroStr);

    % Guarda los cambios en handles
    guidata(hObject, handles);



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function leerDatosUART(hObject, handles)
    global s;
    
    % Leer los datos disponibles en el buffer UART
    if s.NumBytesAvailable > 0
        data = read(s, s.NumBytesAvailable, "char");  % Leer los bytes disponibles
       
        % Comparar el mensaje recibido para ver si se detectó algo o no
        if contains(data, "infrarojoSi")
            % Si se detecta algo, actualizar el text2 para indicar la detección
            set(handles.text20, 'String', 'Detección: Sí');
        elseif contains(data, "infrarojoNo")
            % Si no se detecta nada, actualizar el text2 para indicar que no hay detección
            set(handles.text20, 'String', 'Detección: No');
        end
        %consulta por el estado del cny
        if contains(data, "cnySi")
            % Si se detecta algo, actualizar el text2 para indicar la detección
            set(handles.text21, 'String', 'Detección infr: Sí');
        elseif contains(data, "cnyNo")
            % Si no se detecta nada, actualizar el text2 para indicar que no hay detección
            set(handles.text21, 'String', 'Detección cny: No');    
        end
    end

    % Guardar cambios en handles
    guidata(hObject, handles);
