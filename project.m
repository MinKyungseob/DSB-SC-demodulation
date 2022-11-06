
function varargout = project(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @project_OpeningFcn, ...
                   'gui_OutputFcn',  @project_OutputFcn, ...
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

function project_OpeningFcn(hObject, eventdata, handles, varargin)
handles.output = hObject;

guidata(hObject, handles);

function varargout = project_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;

%슬라이더

function slider1x_Callback(hObject, eventdata, handles)
limit= get(handles.slider1x,'Value');
xlim(handles.axes1, [-limit limit]);

function slider1x_CreateFcn(hObject, eventdata, handles)
    
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider1y_Callback(hObject, eventdata, handles)
limit= get(handles.slider1y,'Value');
ylim(handles.axes1, [-limit limit]);
if(get(handles.radiotri,'Value')==1)
    ylim(handles.axes1, [0 limit]);
end

function slider1y_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function slider2x_Callback(hObject, eventdata, handles)
limit= get(handles.slider2x,'Value');
xlim(handles.axes2, [-limit limit]);



function slider2x_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2y_Callback(hObject, eventdata, handles)
limit= get(handles.slider2y,'Value');
ylim(handles.axes2, [-limit limit]);


function slider2y_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3x_Callback(hObject, eventdata, handles)
limit= get(handles.slider3x,'Value');
xlim(handles.axes3, [-limit limit]);

function slider3x_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3y_Callback(hObject, eventdata, handles)
limit= get(handles.slider3y,'Value');
ylim(handles.axes3, [-0 limit]);

function slider3y_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider4x_Callback(hObject, eventdata, handles)
limit= get(handles.slider4x,'Value');
xlim(handles.axes4, [-limit limit]);

function slider4x_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider4y_Callback(hObject, eventdata, handles)
limit= get(handles.slider4y,'Value');
ylim(handles.axes4, [-0 limit]);

function slider4y_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

%라디오 버튼

% --- Executes on button press in radiosin.
function radiosin_Callback(hObject, eventdata, handles)
if(get(handles.radiosin,'Value')==1)
    set(handles.radiotri,'Value',0);
    mystring= sprintf('사인파');    set(handles.pannel,'Title',mystring);
    mystringx=sprintf('x=sin(2*pi*fm*t)');    set(handles.textx,'String',mystringx);

    set(handles.text13,'Visible',0);
    set(handles.tau,'Visible',0);
end

function radiotri_Callback(hObject, eventdata, handles)
if(get(handles.radiotri,'Value')==1)
    set(handles.radiosin,'Value',0);
    mystring= sprintf('삼각파');   set(handles.pannel,'Title',mystring);
    mystringx=sprintf('x=triangle(tau,-1,1,fs)');    set(handles.textx,'String',mystringx);

    set(handles.text13,'Visible',1);
    set(handles.tau,'Visible',1);
end

%푸쉬 버튼
% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
if(get(handles.radiotri,'Value')==0&&get(handles.radiosin,'Value')==0)
   uiwait(msgbox('message 신호 파형이 선택이 안되어있습니다','Error','error','modal'));
end

df=str2double(get(handles.df,'String')); fs=str2double(get(handles.fs,'String')); fm=str2double(get(handles.fm,'String')); 
fc=str2double(get(handles.fc,'String')); snr=str2double(get(handles.snr,'String')); 
ts=1/fs; T1=-1; T2=1; t=[T1:ts:T2]; N = length(t);

snr_lin=10^(snr/10); % linear SNR

if(get(handles.radiotri,'Value')==1) %삼각파 선택시
    set(handles.radiosin,'Value',0);
    tau=str2double(get(handles.tau,'String')); 
    if(df==0||fs==0||fm==0||fc==0||snr==0||tau==0)
        uiwait(msgbox('입력값들 가운데 0이 있습니다 확인 바랍니다','modal'));
    end
    x=triangle(tau, T1, T2, fs); %triangular, pulse width tau=0.1
    xc=cos(2*pi*fc*t); % carrier signal    
end
if(get(handles.radiosin,'Value')==1) %사인파 선택시
    set(handles.radiotri,'Value',0);
    if(df==0||fs==0||fm==0||fc==0||snr==0)
        uiwait(msgbox('입력값들 가운데 0이 있습니다 확인 바랍니다','modal'));
    end
    x=sin(2*pi*fm*t); xc=cos(2*pi*fc*t);
end
xm=x.*xc; [X,x,df1]=fft_mdf(x,ts,df); % message signal, 푸리에 변환
X=X/fs; f=[0:df1:df1*(length(x)-1)]-fs/2; %frequency vector (range to plot)
[Xm,xm,df1]=fft_mdf(xm,ts,df); % modulated signal 푸리에트랜스폼
Xm=Xm/fs; [XC,xc,df1]=fft_mdf(xc,ts,df); %  carrier 푸리에
signal_power=norm(xm(1:N))^2/N; % power in modulated signal
noise_power=signal_power/snr_lin; % 노이즈 줄이기
noise_std=sqrt(noise_power); % compute noise standard deviation
noise=noise_std*randn(1,length(xm)); % generate noise
r=xm+noise; % add noise to the modulated signal
[R,r,df1]=fft_mdf(r,ts,df); % spectrum of the signal+noise
R=R/fs; % 스케일링
plot(handles.axes1, t,x(1:length(t))); plot(handles.axes2, t,xm(1:length(t))); plot(handles.axes3, f,abs(fftshift(X))); plot(handles.axes4, f,abs(fftshift(Xm)));

%sliderx1 %1/fm 은 message의 1주기
set(handles.slider1x,'Max',min(1,2/fm)); %x 슬라이더 범위 최대 크기 4주기/ 혹은 T의 범위인 -1~1사이 중 작은값으로
set(handles.slider1x,'Min',0.3/fm); %x 슬라이더 범위 최소 0.3주기
set(handles.slider1x,'Value',1/fm);
%slidery1
set(handles.slider1y,'Max',1.3*max(x(1:length(t)))); %y 슬라이더 범위, y값중 가장 큰 값의 1.3배 만큼의 크기 
set(handles.slider1y,'Min',0.3*max(x(1:length(t)))); 
set(handles.slider1y,'Value',max(x(1:length(t))));
%sliderx2
set(handles.slider2x,'Max',min(1,2/fm)); 
set(handles.slider2x,'Min',0.3/fm);
set(handles.slider2x,'Value',1/fm);
%slidery2
set(handles.slider2y,'Max',1.3*max(xm(1:length(t))));  
set(handles.slider2y,'Min',0.3*max(xm(1:length(t))));  
set(handles.slider2y,'Value',max(xm(1:length(t))));
%sliderx3 할 필요가 있을까 해서 보류.(위에처럼 생각나는 방법이 따로 없음)
%slidery3
set(handles.slider3y,'Max',1.3*max(abs(fftshift(X))));
set(handles.slider3y,'Min',0.3*max(abs(fftshift(X))));
set(handles.slider3y,'Value',max(abs(fftshift(X))));
%sliderx4
%slidery4
set(handles.slider4y,'Max',1.3*max(abs(fftshift(Xm))));
set(handles.slider4y,'Min',0.3*max(abs(fftshift(Xm))));
set(handles.slider4y,'Value',max(abs(fftshift(Xm))));



%edit Edit

function snr_Callback(hObject, eventdata, handles)

function snr_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function fc_Callback(hObject, eventdata, handles)

function fc_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function fm_Callback(hObject, eventdata, handles)
% hObject    handle to fm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fm as text
%        str2double(get(hObject,'String')) returns contents of fm as a double


% --- Executes during object creation, after setting all properties.
function fm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function fs_Callback(hObject, eventdata, handles)
% hObject    handle to fm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fm as text
%        str2double(get(hObject,'String')) returns contents of fm as a double


% --- Executes during object creation, after setting all properties.
function fs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function df_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function df_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function pannel_CreateFcn(hObject, eventdata, handles)


function tau_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function tau_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
