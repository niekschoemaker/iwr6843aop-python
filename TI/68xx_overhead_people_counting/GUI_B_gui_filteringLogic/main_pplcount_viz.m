clear, clc, close all

%% SETUP SELECTIONS

% 1. sceneRun needs to be specied.
% Default is to use GUI to setup scene
% To programmatically setup the scene follow the example in Prgm_2box.
 debug = 1;
 discreteStance = 0;
 height_limit = 1.6;
% Scene Run choice: {'GUI_Setup' | 'Prgm_2box' | 'Prgm_MaxFOV','My_Scene'};
% sceneRun = 'GUI_Setup';
sceneRun = 'Prgm_2box';
tColor = [0.5 1 0.5];
stanceDetection = 1;

% 2. Specify COM ports if programmatically setting up scene
if (~strcmp(sceneRun,'GUI_Setup'))
    %%%%% EDIT COM PORTS %%%%%%
    controlSerialPort = 11;
    dataSerialPort = 12;
    loadCfg = 1;
end
fileGUI = 'guiCfg.txt';
guiCfg = fopen(fileGUI, 'r');
tline = fgetl(guiCfg);
k=1;
while ischar(tline)
    config{k} = tline;
    tline = fgetl(guiCfg);
    k = k + 1;
end

sceneRun = config{1};
controlSerialPort = str2num(config{2});
dataSerialPort = str2num(config{3});
safebox = str2num(config{4});
debug = str2num(config{5});
discreteStance = str2num(config{6});
%if using input dialog
% prompt = {'Uart COM', 'Data COM'};
% t2 = 'COM Input'
% dims = [1 35];
% definput = {'11', '12'};
% ans = inputdlg(prompt, t2, dims, definput);
% 
% controlSerialPort = str2num(ans{1});
% dataSerialPort = str2num(ans{2});

p1 = input('UART COM');
p2 = input('DATA COM');
height_limit = input('Enter standing height. Set to 1.6 for default:');
discreteStance = input('Enter 0 for gradient Height, 1 for Stance, 2 for all grey');
dataSerialPort = p2;
controlSerialPort = p1;


%% Setup tracking scene

% Enables setting parameters by GUI
if(strcmp(sceneRun,'GUI_Setup'))
    
    % Call setup GUI
    hSetup = setup();
    close(hSetup.figure1);
    % Get parameters defined in GUI
    camIndex = hSetup.camIndex;
    hDataSerialPort = hSetup.hDataSerialPort;
    hControlSerialPort = hSetup.hControlSerialPort;
    wall = hSetup.wall;
    scene.azimuthTilt = hSetup.angle*pi/180;
    Params = hSetup.params; 
    % Target box settings
    scene.numberOfTargetBoxes = size(hSetup.subzone,1); 
    scene.targetBox = hSetup.subzone;
    % Define wall [BLx BLy W H]
    scene.areaBox = [wall.left wall.back abs(wall.left)+wall.right wall.front+abs(wall.back)];
    
    % Define plotting area [Lx Rx By Ty]
    scene.maxPos = [scene.areaBox(1)-0.1 scene.areaBox(1)+scene.areaBox(3)+0.1 scene.areaBox(2)-0.1 scene.areaBox(2)+scene.areaBox(4)+0.1];
    
    % Chirp config
    loadCfg = 0; %disabled because loaded in GUI
end

% Programmatically set scene. Includes example of setting boundary boxes to count in 
if(strcmp(sceneRun,'Prgm_2box'))
    
    
%     stanceDetection = 1;
    
    %Read Chirp Configuration file
    configurationFileName = 'mmw_pplcount_demo_default.cfg';   
%     configurationFileName = 'test_chirp.cfg';
    cliCfg = readCfg(configurationFileName);
    Params = parseCfg(cliCfg);
    
    cfg.filename = configurationFileName;
    
    hSetup.hControlSerialPort = controlSerialPort;
    hSetup.cfg = cfg;
    
    % Room Wall dimensions [m]
    % Measured relative to radar
    wall.left = -3; % signed: - required
    wall.right = 3;
    wall.front = 3;
    wall.back = -3; % signed: - required
    
    % define wall [BLx BLy W H]
    scene.areaBox = [wall.left wall.back abs(wall.left)+wall.right wall.front+abs(wall.back)];
    
    % Define two rectangles for specific counting in the region
    % Target box settings
    scene.numberOfTargetBoxes = 0;
    
    % Parameters to make it easier to define two rectangles of the same size
        % that are side by side    
    % RO = Rectangle Origin. RO is measured relative to Left Back wall corner
    box.ROxtoLB = 1.0; % x distance from left wall 
    box.ROytoLB = 1.5; % y distance from back wall
    box.height = 2;    % height of boxes 
    box.sep = 0.6;     % seperation width of boxes
    box.width = 1;     % width of boxes 
    box.RTXplot = box.ROxtoLB+wall.left;
    box.RTYplot = box.ROytoLB+wall.back;
    
    
    % Each row of targetBox specifies the dimensions of a rectangle for counting.
    % The # of rows of targetBox must match numberOfTargetBoxes.
    % The rectangles are specified by (x,y) coordinate and width and height 
    % Custom rectangles can be defined instead if two side by side rects of
    % same size are not desired using [RTCx RTCy W H] convention
    scene.targetBox = [box.RTXplot box.RTYplot box.width box.height; 
                       (box.RTXplot+box.width+box.sep) box.RTYplot box.width box.height];
    
    
    % define plotting area as margin around wall
    margin = 0.1; %[m]
    scene.maxPos = [scene.areaBox(1)-margin ...
                    scene.areaBox(1)+scene.areaBox(3)+margin ...
                    scene.areaBox(2)-margin ...
                    scene.areaBox(2)+scene.areaBox(4)+margin];

    % Azimuth tilt of radar. 
    angle = +0; % Signed: + if tilted towards R wall, - if L, 0 if straight forward
    scene.azimuthTilt = angle*pi/180;
end


% Programmatically set scene. Includes example of setting boundary boxes to count in 
if(strcmp(sceneRun,'Prgm_MaxFOV'))
    
    %Read Chirp Configuration file
    configurationFileName = 'mmw_pcdemo_default.cfg';   
    cliCfg = readCfg(configurationFileName);
    Params = parseCfg(cliCfg);
    
    % Room Wall dimensions [m]
    % Measured relative to radar
    wall.left = -6; % signed: - required
    wall.right = 6;
    wall.front = 6;
    wall.back = -0; % signed: - required
    
    % define wall [BLx BLy W H]
    scene.areaBox = [wall.left wall.back abs(wall.left)+wall.right wall.front+abs(wall.back)];
    
    % Define two rectangles for specific counting in the region
    % Target box settings
    scene.numberOfTargetBoxes = 0;
    
    % Parameters to make it easier to define two rectangles of the same size
        % that are side by side    
    % RO = Rectangle Origin. RO is measured relative to Left Back wall corner
    box.ROxtoLB = 1.0; % x distance from left wall 
    box.ROytoLB = 1.5; % y distance from back wall
    box.height = 2;    % height of boxes 
    box.sep = 0.6;     % seperation width of boxes
    box.width = 1;     % width of boxes 
    box.RTXplot = box.ROxtoLB+wall.left;
    box.RTYplot = box.ROytoLB+wall.back;
    
    
    % Each row of targetBox specifies the dimensions of a rectangle for counting.
    % The # of rows of targetBox must match numberOfTargetBoxes.
    % The rectangles are specified by (x,y) coordinate and width and height 
    % Custom rectangles can be defined instead if two side by side rects of
    % same size are not desired using [RTCx RTCy W H] convention
    scene.targetBox = [box.RTXplot box.RTYplot box.width box.height; 
                       (box.RTXplot+box.width+box.sep) box.RTYplot box.width box.height];
    
    
    % define plotting area as margin around wall
    margin = 0.1; %[m]
    scene.maxPos = [scene.areaBox(1)-margin ...
                    scene.areaBox(1)+scene.areaBox(3)+margin ...
                    scene.areaBox(2)-margin ...
                    scene.areaBox(2)+scene.areaBox(4)+margin];

    % Azimuth tilt of radar. 
    angle = +0; % Signed: + if tilted towards R wall, - if L, 0 if straight forward
    scene.azimuthTilt = angle*pi/180;
end

if(strcmp(sceneRun,'My_Scene'))
end

%% Webcam setup
if (strcmp(sceneRun,'GUI_Setup'))
    if(~(camIndex == -1))
        enableWebcam = 1;
        cam = webcam(camIndex);
        resList = cam.AvailableResolution;
        cam.Resolution = resList{getWidestFOV(resList)};


        hWebcamFigure = figure('Name', 'Ground Truth','Tag','webcamFigure',...
            'Toolbar','none', 'Menubar','none',...
            'NumberTitle', 'Off', 'Interruptible', 'Off');
        axWebcam = axes('Parent', hWebcamFigure);
        hImage = image(axWebcam, snapshot(cam));
        axis(axWebcam, 'manual','off')
        

        % Set up the push buttons
        uicontrol('String', 'Play',...
            'Callback', 'preview(cam, hImage)',...
            'Units','normalized',...
            'Position',[0 0 0.15 .07]);
        uicontrol('String', 'Pause',...
            'Callback', 'closePreview(cam)',...
            'Units','normalized',...
            'Position',[.17 0 .15 .07]);
        uicontrol('String', 'Close',...
            'Callback', 'delete(hWebcamFigure)',...
            'Units','normalized',...
            'Position',[0.34 0 .15 .07]);


        axWebcam = axes('Parent', hWebcamFigure);
        hImage = image(axWebcam, snapshot(cam));
        axis(axWebcam, 'manual','off')


        res = cam.Resolution;
        ss = strsplit(res,'x');
        imWidth = str2num(ss{1});
        imHeight = str2num(ss{2});
        hImage = image( zeros(imHeight, imWidth, 3) );
        % Set up the update preview window function.
        setappdata(hImage,'UpdatePreviewWindowFcn',@mypreview_fcn);



        % Specify the size of the axes that contains the image object
        % so that it displays the image at the right resolution and
        % centers it in the figure window.
        figSize = get(hWebcamFigure,'Position');
        figWidth = figSize(3);
        figHeight = figSize(4);
        gca.unit = 'pixels';
        gca.position = [ ((figWidth - imWidth)/2)... 
                       ((figHeight - imHeight)/2)...
                       imWidth imHeight ];


        hCam = preview(cam, hImage);
        pause(0.5); %allow webcam to load
    else
        enableWebcam = 0;
        hWebcamFigure = [];
    end
else
    %Progammatically configure webcam here
    enableWebcam = 0;
    hWebcamFigure = [];
end


%% Serial setup
if (~strcmp(sceneRun,'GUI_Setup'))
    %Configure data UART port with input buffer to hold 100+ frames 
    hDataSerialPort = configureDataSport(dataSerialPort, 65536);
    
    %Send Configuration Parameters to IWR16xx
    if(loadCfg)
        mmwDemoCliPrompt = char('mmwDemo:/>');
        hControlSerialPort = configureControlPort(controlSerialPort);
        hSetup.hControlSerialPort = hControlSerialPort;
        %Send CLI configuration to IWR16xx
        fprintf('Sending configuration from %s file to IWR16xx ...\n', configurationFileName);
        for k=1:length(cliCfg)
            fprintf(hControlSerialPort, cliCfg{k});
            fprintf('%s\n', cliCfg{k});
            echo = fgetl(hControlSerialPort); % Get an echo of a command
            done = fgetl(hControlSerialPort); % Get "Done" 
            prompt = fread(hControlSerialPort, size(mmwDemoCliPrompt,2)); % Get the prompt back 
        end
%         fclose(hControlSerialPort);
%         delete(hControlSerialPort);
    end
end

%% Init variables
trackerRun = 'Target';
colors='brgcm';
labelTrack = 0;

%sensor parameters
%sensor.rangeMax = 6;
sensor.rangeMax = Params.dataPath.numRangeBins*Params.dataPath.rangeIdxToMeters;
sensor.rangeMin = 1;
sensor.azimuthFoV = 120*pi/180; %120 degree FOV in horizontal direction
sensor.framePeriod = Params.frameCfg.framePeriodicity;
sensor.maxURadialVelocity = 20;
sensor.angles = linspace(-sensor.azimuthFoV/2, sensor.azimuthFoV/2, 128);

hTargetBoxHandle = [];
peopleCountTotal = 0;
peopleCountInBox = zeros(1, scene.numberOfTargetBoxes);
rxData = zeros(10000,1,'uint8');

maxNumTracks = 20;
maxNumPoints = 250;

hPlotCloudHandleAll = [];
hPlotCloudHandleOutRange = [];
hPlotCloudHandleClutter = [];
hPlotCloudHandleStatic = [];
hPlotCloudHandleDynamic =[];
hPlotPoints3D = [];

clutterPoints = zeros(2,1);
activeTracks = zeros(1, maxNumTracks);
overlap = zeros(1, maxNumTracks);
%0 if no overlap
% if not zero, value is tid of the blocking track
sleepingTracks = zeros(1, maxNumTracks);
%tracks that are sleeping (not reported by tracker)
%0 means no sleeping track
%value > 0 denotes sleeping and time in frames its been asleep
sleepingShat = zeros(9, 20);
sleepingColor = zeros(3, maxNumTracks);
trackState = zeros(1, maxNumTracks);
trackAge = zeros(1, maxNumTracks);
%states 
% 1 = New
% 2 = Established
% 3 = No Returned Track
% 0 = No track
%safebox = [-2 2 -0.5 2];
sitting = 'b';
standing = 'g';
age2Est = 50;
avLength = 20;
height_vals = zeros(20, avLength);
stance_hist = zeros(20, avLength);
stance_current = zeros(1, 20);
sleepingStance = zeros(1, 20);

trackingHistStruct = struct('tid', 0, 'allocationTime', 0, 'tick', 0, 'posIndex', 0, 'histIndex', 0, 'sHat', zeros(1000,9), 'ec', zeros(1000,16),'pos', zeros(100,3), 'hMeshU', [], 'hMeshG', [], 'hPlotAssociatedPoints', [], 'hPlotTrack', [], 'hPlotCentroid', []);
trackingHist = repmat(trackingHistStruct, 1, maxNumTracks);

%% Setup figure

figHandle = figure('Name', 'Visualizer','tag','mainFigure');


clf(figHandle);
set(figHandle, 'WindowStyle','normal');
set(figHandle,'Name','Texas Instruments - People Counting','NumberTitle','off')    

set(figHandle,'currentchar',' ')         % set a dummy character

warning off MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame
jframe=get(figHandle,'javaframe');
set(figHandle, 'MenuBar', 'none');
set(figHandle, 'Color', [0 0 0]);
%set(figHandle, 'CloseRequestFcn', close(figHandle));
%set(figHandle, 'DeleteFcn', @close_main);
pause(0.00001);
set(jframe,'Maximized',1); 
pause(0.00001);


% Background


figureTitles = {'Legend', 'Gating and Association', 'Chirp Configuration', 'Visualizer Options & Control', 'Statistics'}; %, 'Heat Map', 'Feature Ext'};
figureGroup = [1, 3, 1, 1, 1];
numFigures = size(figureTitles, 2);
hFigure = zeros(1,numFigures);

hTabGroup(1) = uitabgroup(figHandle, 'Position', [0.0 0 0.3 1]);

hTabGroup(3) = uitabgroup(figHandle, 'Position', [0.3 0 0.7 1]); 

% if((wall.right-wall.left) > (wall.front - wall.back))
%    hTabGroup(2) = uitabgroup(figHandle, 'Position', [0.2 0.5 0.8 0.5]);
%    hTabGroup(3) = uitabgroup(figHandle, 'Position', [0.2 0.0 0.4 0.1]); 
% else
%    hTabGroup(2) = uitabgroup(figHandle, 'Position', [0.2 0 0.4 1]);
%    
% end

for iFig = 1:numFigures
    hFigure(iFig) = uitab(hTabGroup(figureGroup(iFig)), 'Title', figureTitles{iFig});

%     if(strcmp(figureTitles{iFig},'Point Cloud'))
%         trackingAx = axes('parent', hFigure(iFig));
% 
%         %plot(trackingAx, sensor.rangeMin*sin(sensor.angles+scene.azimuthTilt), sensor.rangeMin*cos(sensor.angles+scene.azimuthTilt), '-k');  hold on;
%         %plot(trackingAx, [0 sensor.rangeMax*sin(sensor.angles+scene.azimuthTilt) 0],[0 sensor.rangeMax*cos(sensor.angles+scene.azimuthTilt) 0], '-k');
%   
%         title(figureTitles{iFig},'FontUnits','Normalized', 'FontSize',0.05);
%         axis equal;
%         axis(scene.maxPos);
%         camroll(180)
%         
%         % draw wall box
%         rectangle(trackingAx, 'Position', scene.areaBox, 'EdgeColor','k', 'LineStyle', '-', 'LineWidth', 2);
%         
%         % draw target box
%         for nBoxes = 1:scene.numberOfTargetBoxes
%             hTargetBoxHandle(nBoxes)= rectangle('Parent', trackingAx, 'Position', scene.targetBox(nBoxes,:), 'EdgeColor', colors(nBoxes), 'LineWidth', 4);
%         end
%         
%         
%         grid on;
%         hold on;
%         grid minor;
% 
%     end
    
    if(strcmp(figureTitles{iFig},'Gating and Association'))
        gatingAx = axes('parent', hFigure(iFig));

        cax = [1.0 2];
        if (discreteStance == 0)
            title({'Overhead Tracking'},'FontUnits','Normalized', 'FontSize',0.05);
        else
            title({'People Stance Detection';''},'FontUnits','Normalized', 'FontSize',0.05); %deb_gp 
        end
        axis equal;
        axis(scene.maxPos);        
        if (discreteStance == 0)
            cbar = colorbar('Ticks', [1.25, 1.75], 'TickLabels', {'Sitting', 'Standing'});
        end
        caxis([1 2]);
       
        camroll(180)
        
        % draw wall box
        rectangle(gatingAx, 'Position', scene.areaBox, 'EdgeColor','k', 'LineStyle', '-', 'LineWidth', 2);
        
        grid on;
        hold on;
        grid minor;


    end
    
    if(strcmp(figureTitles{iFig},'Chirp Configuration'))
        %axes('parent', hFigure(iFig))
        tablePosition = [0.1 0.45 0.8 0.5];
        displayChirpParams(Params, tablePosition, hFigure(iFig));
    end
    
    if (strcmp(figureTitles{iFig}, 'Legend'))
        axes('parent', hFigure(iFig));
        image(imread('Legend2.png'));
        axis off;
    end
    
    if(strcmp(figureTitles{iFig},'Statistics'))     
        axes('parent', hFigure(iFig))
        hStatGlobal(1) = text(0, 0.9, 'Frame # 0', 'FontSize',12, 'Visible', 'on');
        hStatGlobal(2) = text(0, 0.8, 'Detection Points: 0','FontSize',12, 'Visible', 'on');
        hStatGlobal(3) = text(0, 0.6, 'People Count:  0','FontSize',24);
        hStatGlobal(4) = text(0, 0.4, 'Stance:  Unknown', 'FontSize', 18); 
        for i=1:scene.numberOfTargetBoxes
            hStatGlobal(end+1) = text(0, 0.6-0.15*i, 'Box Count','FontSize', 30, 'color', colors(i));
        end
        hStatGlobal(end+1) = text(0, 0.7, 'Bytes Available:  0/0','FontSize',12, 'Visible', 'on');
        axis off;
    end
    
    if(strcmp(figureTitles{iFig},'Visualizer Options & Control'))
        cFig = iFig;
        contW = 0.75;
        contH = 0.05;
        %imshow('tiLogo.jpg')
        hRbPause = uicontrol(hFigure(cFig),'style','checkbox', 'string', 'Pause',...
            'Units','Normalized', 'Position',[0.05 0.85 contW contH],...
            'FontSize', 15);
        
        hPlotTabs = uicontrol(hFigure(cFig),'style','checkbox','string','Consolidate plotting tabs',...
             'Units','Normalized', 'Position',[0.05 0.9 contW contH],'Value',0,...
             'FontSize', 15, 'Callback', {@checkPlotTabs,hTabGroup});

        hPbExit = uicontrol('Style', 'pushbutton', 'String', 'EXIT',...
            'Position', [10 10 100 40],'Callback', @exitPressFcn);
        setappdata(hPbExit, 'exitKeyPressed', 0);
        
        hResendCfg = uicontrol('Style', 'pushbutton', 'String', 'Resend Cfg',...
            'Position', [10 80 100 40], 'UserData', hSetup, 'Callback', @resendPressFcn);
        setappdata(hResendCfg, 'ResendPressed', 0);
    end    
end


%% Data structures
syncPatternUINT64 = typecast(uint16([hex2dec('0102'),hex2dec('0304'),hex2dec('0506'),hex2dec('0708')]),'uint64');
syncPatternUINT8 = typecast(uint16([hex2dec('0102'),hex2dec('0304'),hex2dec('0506'),hex2dec('0708')]),'uint8');

frameHeaderStructType = struct(...
    'sync',             {'uint64', 8}, ... % See syncPatternUINT64 below
    'version',          {'uint32', 4}, ...
    'platform',         {'uint32', 4}, ...
    'timestamp',        {'uint32', 4}, ... % 600MHz clocks
    'packetLength',     {'uint32', 4}, ... % In bytes, including header
    'frameNumber',      {'uint32', 4}, ... % Starting from 1
    'subframeNumber',   {'uint32', 4}, ...
    'chirpMargin',      {'uint32', 4}, ... % Chirp Processing margin, in ms
    'frameMargin',      {'uint32', 4}, ... % Frame Processing margin, in ms
    'uartSentTime' ,    {'uint32', 4}, ... % Time spent to send data, in ms
    'trackProcessTime', {'uint32', 4}, ... % Tracking Processing time, in ms
    'numTLVs' ,         {'uint16', 2}, ... % Number of TLVs in thins frame
    'checksum',         {'uint16', 2});    % Header checksum

tlvHeaderStruct = struct(...
    'type',             {'uint32', 4}, ... % TLV object Type
    'length',           {'uint32', 4});    % TLV object Length, in bytes, including TLV header 

% Point Cloud TLV object consists of an array of points. 
% Each point has a structure defined below
pointStruct = struct(...
    'range',            {'float', 4}, ... % Range, in m
    'angle',            {'float', 4}, ... % Angel, in rad
    'elev',             {'float', 4}, ...
    'doppler',          {'float', 4}, ... % Doplper, in m/s
    'snr',              {'float', 4});    % SNR, ratio
% Target List TLV object consists of an array of targets. 
% Each target has a structure define below
% targetStruct = struct(...
%     'tid',              {'uint32', 4}, ... % Track ID
%     'posX',             {'float', 4}, ... % Target position in X dimension, m
%     'posY',             {'float', 4}, ... % Target position in Y dimension, m
%     'posZ',             {'float', 4}, ...
%     'velX',             {'float', 4}, ... % Target velocity in X dimension, m/s
%     'velY',             {'float', 4}, ... % Target velocity in Y dimension, m/s
%     'velZ',             {'float', 4}, ...
%     'accX',             {'float', 4}, ... % Target acceleration in X dimension, m/s2
%     'accY',             {'float', 4}, ... % Target acceleration in Y dimension, m/s
%     'accZ',             {'float', 4}, ...
%     'EC',               {'float', 16*4}, ... % Tracking error covariance matrix, [3x3], in range/angle/doppler coordinates
%     'G',                {'float', 4});    % Gating function gain

targetStruct = struct(...
    'tid',              {'uint32', 4}, ... % Track ID
    'posX',             {'float', 4}, ... % Target position in X dimension, m
    'posY',             {'float', 4}, ... % Target position in Y dimension, m
    'posZ',             {'float', 4}, ...
    'velX',             {'float', 4}, ... % Target velocity in X dimension, m/s
    'velY',             {'float', 4}, ... % Target velocity in Y dimension, m/s
    'velZ',             {'float', 4}, ...
    'accX',             {'float', 4}, ... % Target acceleration in X dimension, m/s2
    'accY',             {'float', 4}, ... % Target acceleration in Y dimension, m/s
    'accZ',             {'float', 4});
%     'EC',               {'float', 16*4}, ... % Tracking error covariance matrix, [3x3], in range/angle/doppler coordinates
%     'G',                {'float', 4});    % Gating function gain

frameHeaderLengthInBytes = lengthFromStruct(frameHeaderStructType);
tlvHeaderLengthInBytes = lengthFromStruct(tlvHeaderStruct);
pointLengthInBytes = lengthFromStruct(pointStruct);
targetLengthInBytes = lengthFromStruct(targetStruct);
indexLengthInBytes = 1;

exitRequest = 0;
lostSync = 0;
gotHeader = 0;
outOfSyncBytes = 0;
runningSlow = 0;
maxBytesAvailable = 0;
point3D = [];

frameStatStruct = struct('targetFrameNum', [], 'bytes', [], 'numInputPoints', 0, 'numOutputPoints', 0, 'timestamp', 0, 'start', 0, 'benchmarks', [], 'done', 0, ...
    'pointCloud', [], 'targetList', [], 'indexArray', [], 'height', []);
fHist = repmat(frameStatStruct, 1, 10000);
%videoFrame = struct('cdata',[],'colormap', []);
%F = repmat(videoFrame, 10000,1);
optimize = 1;
skipProcessing = 0;
frameNum = 1;
frameNumLogged = 1;
fprintf('------------------\n');

%num_meas = zeros(20, 1);

update = 0;
%lostSyncTime = 0;
%% Main
while(isvalid(hDataSerialPort))

    p1height = -1;
    stance = "Unknown";
    while(lostSync == 0 && isvalid(hDataSerialPort))

        frameStart = tic;
        fHist(frameNum).timestamp = frameStart;
        bytesAvailable = get(hDataSerialPort,'BytesAvailable');
        if(bytesAvailable > maxBytesAvailable)
            maxBytesAvailable = bytesAvailable;
        end
        fHist(frameNum).bytesAvailable = bytesAvailable;
        if(gotHeader == 0)
            %Read the header first
            [rxHeader, byteCount] = fread(hDataSerialPort, frameHeaderLengthInBytes, 'uint8');
        end
        fHist(frameNum).start = 1000*toc(frameStart);
        
        magicBytes = typecast(uint8(rxHeader(1:8)), 'uint64');
        if(magicBytes ~= syncPatternUINT64)
            reason = 'No SYNC pattern';
            lostSync = 1;
            break;
        end
        if(byteCount ~= frameHeaderLengthInBytes)
            reason = 'Header Size is wrong';
            lostSync = 1;
            break;
        end        
        if(validateChecksum(rxHeader) ~= 0)
            reason = 'Header Checksum is wrong';
            lostSync = 1;
            break; 
        end
        
        frameHeader = readToStruct(frameHeaderStructType, rxHeader);
        
        if(gotHeader == 1)
            if(frameHeader.frameNumber > targetFrameNum)
                targetFrameNum = frameHeader.frameNumber;
                disp(['Found sync at frame ',num2str(targetFrameNum),'(',num2str(frameNum),'), after ', num2str(1000*toc(lostSyncTime),3), 'ms']);
                gotHeader = 0;
            else
                reason = 'Old Frame';
                gotHeader = 0;
                lostSync = 1;
                break;
            end
        end
        
        % We have a valid header
        targetFrameNum = frameHeader.frameNumber;
        fHist(frameNum).targetFrameNum = targetFrameNum;
        fHist(frameNum).header = frameHeader;
        
        dataLength = frameHeader.packetLength - frameHeaderLengthInBytes;
        
        fHist(frameNum).bytes = dataLength; 
        numInputPoints = 0;
        numTargets = 0;
        mIndex = [];

        if(dataLength > 0)
            %Read all packet
            [rxData, byteCount] = fread(hDataSerialPort, double(dataLength), 'uint8');
            if(byteCount ~= double(dataLength))
                reason = 'Data Size is wrong'; 
                lostSync = 1;
                break;  
            end
            offset = 0;
    
            fHist(frameNum).benchmarks(1) = 1000*toc(frameStart);

            % TLV Parsing
            for nTlv = 1:frameHeader.numTLVs
                tlvType = typecast(uint8(rxData(offset+1:offset+4)), 'uint32');
                tlvLength = typecast(uint8(rxData(offset+5:offset+8)), 'uint32');
                if(tlvLength + offset > dataLength)
                    reason = 'TLV Size is wrong';
                    lostSync = 1;
                    break;                    
                end
                offset = offset + tlvHeaderLengthInBytes;
                valueLength = tlvLength - tlvHeaderLengthInBytes;
                switch(tlvType)
                    case 6
                        % Point Cloud TLV
                        numInputPoints = valueLength/pointLengthInBytes;
                        if(numInputPoints > 0)                        
                            % Get Point Cloud from the sensor
                            p = typecast(uint8(rxData(offset+1: offset+valueLength)),'single');

                            pointCloud = reshape(p,5, numInputPoints);    
%                            pointCloud(2,:) = pointCloud(2,:)*pi/180;
                            xyz = zeros(3, numInputPoints);
                            xyz(3, :) = pointCloud(1,:).*sin(pointCloud(3,:));
                            r = pointCloud(1,:).*cos(pointCloud(3,:));
                            xyz(2, :) = r.*cos(pointCloud(2,:));
                            xyz(1, :) = r.*sin(pointCloud(2,:));

                            posAll = [xyz(1,:); xyz(3,:)];
                            snrAll = pointCloud(5,:);

                            % Remove out of Range, Behind the Walls, out of FOV points
                            inRangeInd = (pointCloud(1,:) > 1) & (pointCloud(1,:) < 6) & ...
                                (pointCloud(2,:) > -50*pi/180) &  (pointCloud(2,:) < 50*pi/180) & ...
                                (posAll(1,:) > scene.areaBox(1)) & (posAll(1,:) < (scene.areaBox(1) + scene.areaBox(3))) & ...
                                (posAll(2,:) > scene.areaBox(2)) & (posAll(2,:) < (scene.areaBox(2) + scene.areaBox(4)));
                            pointCloudInRange = pointCloud(:,inRangeInd);
                            posInRange = posAll(:,:);
                            numOutputPoints = size(pointCloud,2);                          
                        end                        
                        offset = offset + valueLength;
                                            
                    case 7
                        % Target List TLV
                        numTargets = valueLength/targetLengthInBytes;                        
                        TID = zeros(1,numTargets);
                        S = zeros(9, numTargets);
                        EC = zeros(16, numTargets);
                        G = zeros(1,numTargets);                        
                        for n=1:numTargets
                            TID(n)  = typecast(uint8(rxData(offset+1:offset+4)),'uint32');      %1x4=4bytes
                            if (floor(TID(n)) ~= TID(n) || TID(n) > 20)
                                lostSync = 1;
                                break;
                            end
                            S(:,n)  = typecast(uint8(rxData(offset+5:offset+40)),'single');     %9x4=36bytes
%                             EC(:,n) = [];
%                             G(n) = [];
%                             EC(:,n) = typecast(uint8(rxData(offset+41:offset+104)),'single');    %16x4=64bytes
%                             G(n)    = typecast(uint8(rxData(offset+105:offset+108)),'single');    %1x4=4bytes
                            offset = offset + 40;
                        end
                        
                    case 8
                        % Target Index TLV
                        numIndices = valueLength/indexLengthInBytes;
                        mIndex = typecast(uint8(rxData(offset+1:offset+numIndices)),'uint8');
                        offset = offset + valueLength;
                        tind = unique(mIndex);
                        if (stanceDetection)
                            if (frameNum - 1 == 0 || isempty(fHist(frameNum-1).height))
                                fHist(frameNum).height = zeros(20, 1);
                            else
                                fHist(frameNum).height = fHist(frameNum-1).height;
                            end
                            if (size(point3D, 2) == size(mIndex, 1))
                                for i=1:size(tind)
                                    if (tind(i) < 30) 
                                        h_ind = mIndex == tind(i);
                                        h_ind = point3D(3, h_ind);
                                        d_ind = h_ind(1, :) > 0.5 & h_ind(1, :) < 3;
                                        h_ind = h_ind(1, d_ind);
                                        if (size(h_ind))
                                            height_vals(tind(i) + 1, mod(frameNum, avLength) + 1) = 3 - min(h_ind);
                                        else
                                            height_vals(tind(i) + 1, mod(frameNum, avLength) + 1) = 1.5;
                                        end
                                        fHist(frameNum).height(tind(i)+1) = getHeight(tind(i), height_vals);
                                        
                                    end
                                end
                            end
                        end                        
                end
            end
        end
        if (lostSync == 1)
            break;
        end
       
        if(numInputPoints == 0)
            numOutputPoints = 0;
            pointCloud = single(zeros(4,0));
            posAll = [];
            xyz = zeros(3,0);
            posInRange = [];  
        end
        if(numTargets == 0)
            TID = [];
            S = [];
            EC = [];
            G = [];
        end
        
        fHist(frameNum).numInputPoints = numInputPoints;
        fHist(frameNum).numOutputPoints = numOutputPoints;    
        fHist(frameNum).numTargets = numTargets;
        fHist(frameNum).pointCloud = pointCloud;
        fHist(frameNum).targetList.numTargets = numTargets;
        fHist(frameNum).targetList.TID = TID;
        fHist(frameNum).targetList.S = S;
        if(~optimize)
            fHist(frameNum).targetList.EC = EC;
        end
        fHist(frameNum).targetList.G = G;
        fHist(frameNum).indexArray = mIndex;

       
        % Plot pointCloud
        fHist(frameNum).benchmarks(2) = 1000*toc(frameStart);
   
        if(get(hRbPause, 'Value') == 1)
            pause(0.01);
            continue;
        end
        
        % Delete previous points
        if(ishandle(hPlotCloudHandleAll))
            delete(hPlotCloudHandleAll);
        end
        if(ishandle(hPlotCloudHandleOutRange))
            delete(hPlotCloudHandleOutRange);
        end
        if(ishandle(hPlotCloudHandleClutter))
            delete(hPlotCloudHandleClutter);
        end
        if(ishandle(hPlotCloudHandleStatic))
            delete(hPlotCloudHandleStatic);
        end
        if(ishandle(hPlotCloudHandleDynamic))
            delete(hPlotCloudHandleDynamic);
        end

%         if(size(posAll,2))
%             % Plot all points
%             if(sum(snrAll) >= 0)
%                 if(~optimize)
%                     hPlotCloudHandleAll = scatter(trackingAx, posAll(1,:), posAll(2,:),'.k','SizeData',snrAll*10);
%                 else
%                     hPlotCloudHandleAll = plot(trackingAx, posAll(1,:), posAll(2,:),'.k', 'MarkerSize', 20);
%                 end         
%             end
%             % Cross out out-of-Range
%             if(~optimize)
%                 hPlotCloudHandleOutRange = plot(trackingAx, posAll(1,~inRangeInd), posAll(2,~inRangeInd), 'xr');
%             end
%         end        
        fHist(frameNum).benchmarks(3) = 1000*toc(frameStart);

        switch trackerRun
            case 'Target'
                if(numTargets == 0)
                    TID = zeros(1,0);
                    S = zeros(6,0);
                    EC = zeros(9,0);
                    G = zeros(1,0);
                end
        end
        
        fHist(frameNum).benchmarks(4) = 1000*toc(frameStart);
        
        if nnz(isnan(S))
            reason = 'Error: S contains NaNs';
            lostSync = 1;
            break;
        end
%         if nnz(isnan(EC))
%             reason = 'Error: EC contains NaNs';
%             lostSync = 1;
%             break;
%         end
        
        tNumC = length(TID);
        peopleCountTotal = tNumC;
        peopleCountInBox = zeros(1, scene.numberOfTargetBoxes);
 
        if(size(mIndex,1)) 
            mIndex = mIndex + 1;
        end
        
        % Plot previous frame's 3D points       
%         if(size(point3D,2))   
%             if isempty(hPlotPoints3D)
%                 %hPlotPoints3D = plot(gatingAx, point3D(1,:), point3D(2,:), '.k');%, point3D(3,:),'.k');
%             else
%                 %set(hPlotPoints3D, 'XData', point3D(1,:),'YData', point3D(2,:)); %, 'ZData', point3D(3,:));
%             end
%         end     
        for n=1:tNumC
            tid = TID(n)+1;
            dim = 0.3;
            if(activeTracks(tid) == 0)
                %check to see if new track replaces sleeping track or if
                %new track overlaps with existing track
                activeTracks(tid) = 1;
                trackAge(tid) = 1;
                trackState(tid) = 1;
                %overlap position is different based on location
%                 if (isInBox(S(:,n), safebox) == 1)
%                     dim = 0.6;
%                 else
%                     dim = 0.3;
%                 end
                %check for overlap    
                for m = 1:tNumC
                    if (m ~= n)
                        if (hypot(S(1,n) - S(1,m), S(3,n)-S(3,m)) < 2*dim)
                            if (activeTracks(tid) ~= 0)% && overlap(m) == 0)
                                overlap(tid) = TID(m)+1;
                                break;
                            end
                        end
                    end
                end
                %check for sleepers if this track is not an overlap
                if (overlap(tid) == 0)
                    [sleepingTracks, woke, im] = checkFreeSleepers(n, sleepingTracks, sleepingShat, S);
                    if (im > 0)
                        if (sleepingStance(im) == 1)
                            stance_current(tid) = 1;
                            stance_hist(tid, :) = 1;
                        end
                        sleepingStance(im) = 0;
                    end
                    if (woke == 1 && overlap(tid) == 0)
                        trackAge(tid) = 11;
                        trackState(tid) = 2;
                    end
                end
            else
                %if track already exists increment age and update state -
                %check to see if track is no longer an overlap - if so,
                %remove overlap status and check to see if any sleeping
                %tracks should be removed
                activeTracks(tid) = 1; 
                trackAge(tid) = trackAge(tid) + 1;
                if (trackAge(tid) < 10)
                    %check for overlap    
                for m = 1:tNumC
                    if (m ~= n)
                        if (hypot(S(1,n) - S(1,m), S(3,n)-S(3,m)) < 2*dim)
                            if (activeTracks(tid) ~= 0)% && overlap(m) == 0)
                                overlap(tid) = TID(m)+1;
                                break;
                            end
                        end
                    end
                end
                end
                if (trackAge(tid) > age2Est && overlap(tid) == 0)
                    trackState(tid) = 2;
                end
                if (overlap(tid) > 0)
                    %find the track that blocked this from spawning
                    blocker = find(TID == (overlap(tid) - 1));
                    if (isempty(blocker))
                        %find other tracks that were blocked by this track,
                        %overlap index is tid, one of these tracks will
                        %replace it
                        otherVics = find(overlap == overlap(tid));
                        strongest = -1;
                        sCount = 0;
                        %which track has most points?
                        for i=1:size(otherVics, 2)
                            sc = sum(mIndex == otherVics(i));
                            if (sc > sCount)
                                strongest = otherVics(i);
                                sCount = sc;
                            end
                        end
                        if (strongest == -1)
                            strongest = tid;
                        end
                        trackState(strongest) = trackState(overlap(tid));
                        %if blocker is no longer reported, ensure it is not
                        %marked as sleeper
                        trackState(overlap(tid)) = 0;
                        trackAge(overlap(tid)) = 0;
                        %mark all as overlapped by strongest
                        for i = 1:size(otherVics, 2)
                           overlap(otherVics(i)) = strongest; 
                        end
                        %strongest is overlapped by no one
                        overlap(strongest) = 0;
                    end
                end
            end
            if((overlap(tid) == 0 && debug == 0) || (debug == 1))                
                if(tid > maxNumTracks)
                    reason = 'Error: TID is wrong';
                    lostSync = 1;
                    break;
                end
                dim = 0.3;
                %cmap = colormap(cbar);
                if( (size(mIndex,1) > 0) && (size(mIndex,1) == size(point3D,2)) )
                    if (stanceDetection)                       
                        h = fHist(frameNum).height(tid);
                        %discrete stance detection
                        if (discreteStance == 1)
                            if (h > height_limit)
                                stance_hist(tid, mod(frameNum, avLength) + 1) = 2;
                            else
                                stance_hist(tid, mod(frameNum, avLength) + 1) = 1;
                            end
                            st = sum(stance_hist(tid, :) == 2);
                            si = sum(stance_hist(tid, :) == 1);
                            if (st > 0.7 * avLength)
                                stance_current(tid) = 2;
                            elseif (si > 0.7*avLength)
                                stance_current(tid) = 1;
                            else
                                stance_current(tid) = 0;
                            end
                            if(stance_current(tid) == 2)
                                tColor = standing;
                            elseif(stance_current(tid) == 1)
                                tColor = sitting;
                            else
                                tColor = [0.5, 0.5, 0.5];
                            end
                        elseif (discreteStance == 0) %gradient height detection
                            cmap = colormap(cbar);
                            tColor = round(((fHist(frameNum).height(tid) - cax(1))/(cax(2) - cax(1))) * size(cmap,1));
                            if (tColor > size(cmap, 1))
                                tColor = size(cmap, 1);
                            end
                            if (tColor < 1)
                                tColor = 1;
                            end
                            tColor = cmap(tColor, :);
                        else
                            tColor = [0.5 0.5 0.5];
                        end
                    else
                        tColor = colors(1);
                    end
                    if (overlap(tid) > 0)
                        tColor = 'r';
                    end
                    %sleepingColor(:, tid) = tColor;
                    %ind = (mIndex == tid);
                    %valid = point3D(:, ind);
                    %plot point cloud on gating plot - currently removed
%                     ind = (hypot(S(1,n) - valid(1,:), S(3,n) - valid(2, :)) <= dim);
%                     if nnz(ind)
%                         %trackingHist(tid).hPlotAssociatedPoints = plot(gatingAx, point3D(1,ind), point3D(2,ind), 'o', 'color', tColor)
%                         if (isempty(trackingHist(tid).hPlotAssociatedPoints) || ~ishandle(trackingHist(tid).hPlotAssociatedPoints))
%     %                         trackingHist(tid).hPlotAssociatedPoints = plot(gatingAx, point3D(1,ind), point3D(2,ind), '.', 'MarkerSize', 10, 'color', tColor);
%                             trackingHist(tid).hPlotAssociatedPoints = plot(gatingAx, valid(1,ind), valid(2,ind), '.', 'MarkerSize', 10, 'color', tColor);
%                         else
%                             if ishandle(trackingHist(tid).hPlotAssociatedPoints)
%     %                             set(trackingHist(tid).hPlotAssociatedPoints, 'XData', point3D(1,ind),'YData', point3D(2,ind), 'color', tColor);
%                                 set(trackingHist(tid).hPlotAssociatedPoints, 'XData', valid(1,ind),'YData', valid(2,ind), 'color', tColor);
%                             end
%                         end
%                     end
                end

                centroid = computeH(1, S(:,n));
%                 ec = reshape(EC(:,n),4,4);
                if(1)
                    if isempty(trackingHist(tid).hMeshU)
                        if (trackState(tid) == 1)
                            LS = '--';
                        else
                            LS = '-';
                        end
                        trackingHist(tid).hMeshU = circle(gatingAx,S(1,n), S(3,n),dim, LS);
                        if(labelTrack)
                            trackingHist(tid).hMeshU.UserData = text(gatingAx,S(1,n), S(3,n), char(65+mod(tid,26)),'HorizontalAlignment','center','FontUnits','normalized','FontSize',0.5/scene.maxPos(4)*0.75);
                        end
                        if(~optimize)
                            trackingHist(tid).hMeshU.EdgeColor = [0.5 0.5 0.5];
                        else
                            trackingHist(tid).hMeshU.EdgeColor = tColor;
                        end
                        if (trackState(tid) == 2)
                            trackingHist(tid).hMeshU.FaceColor = tColor;
                        else
                            trackingHist(tid).hMeshU.FaceColor = 'none';
                        end
                    else
                        if (trackState(tid) == 2 && overlap(tid) == 0)
                             trackingHist(tid).hMeshU.FaceColor = tColor;
                             trackingHist(tid).hMeshU.LineStyle = '-';
                        end
                        trackingHist(tid).hMeshU.Position = updateCenter(S(1,n), S(3,n),dim);
                        trackingHist(tid).hMeshU.EdgeColor = tColor;
                       
                        if(labelTrack)
                            trackingHist(tid).hMeshU.UserData.Position = [S(1,n), S(3,n)];
                        end
                    end
                end

                if(~optimize)
                    if(g ~= 0)
                        [xG, yG, zG, vG] = gatePlot3(gatingAx, g, centroid, ec);
                        if isempty(trackingHist(tid).hMeshG)
                            trackingHist(tid).hMeshG = mesh(gatingAx, xG.*sin(yG),xG.*cos(yG), zG);
    %                         trackingHist(tid).hMeshG.EdgeColor = colors(mod(tid,length(colors))+1);
                            trackingHist(tid).hMeshG.EdgeColor = colors(1);
                            trackingHist(tid).hMeshG.FaceColor = 'none';
                        else
                            %if (trackingHist(tid).MeshG.FaceColor == 'none'
                            set(trackingHist(tid).hMeshG, 'XData', xG.*sin(yG),'YData',xG.*cos(yG), 'ZData', zG);
                        end
                    end
                end
            end
        end
        
        inds = find(sleepingTracks > 0);
        tC = 10;
        for n = 1:size(inds, 2)
            in = inds(n);
            if isempty(trackingHist(in).hMeshU)
                trackingHist(in).hMeshU = circle(gatingAx,sleepingShat(1,in), sleepingShat(3,in),dim, '-');
                trackingHist(in).hMeshU.FaceColor = 'none';
                if (sleepingStance(in) == 1)
                    trackingHist(in).hMeshU.EdgeColor = sitting;
                else
                    trackingHist(in).hMeshU.EdgeColor = [0.0, 0.0, 0.0]+0.05*tC;
                end
                
%             else
%                 trackingHist(in).hMeshU.Position = updateCenter(sleepingShat(1,in), sleepingShat(3,in),dim);
%                 if (sleepingStance(in) == 1)
%                     trackingHist(in).hMeshU.EdgeColor = sitting;
%                 else
%                     trackingHist(in).hMeshU.EdgeColor = [0.0, 0.0, 0.0]+0.05*tC;
%                 end
            end
        end
        %if track is no longer reported, delete if near edge of zone,
        %otherwise, it becomes a sleeping track, which will be deleted
        %after some number of frames
        iDelete = find(activeTracks == 2);

        for n=1:length(iDelete)
            ind = iDelete(n);
            if (~isempty(fHist(frameNum-1).targetList))
                tid = find(fHist(frameNum-1).targetList.TID == (ind-1));
                if (overlap(ind) == 0 & trackState(ind) == 2 & isInBox(fHist(frameNum-1).targetList.S(:, tid), safebox) && hypot(fHist(frameNum-1).targetList.S(4, tid), fHist(frameNum-1).targetList.S(6, tid)) < 1)
                    sleepingTracks(ind+10) = 1;
                    sleepingStance(ind+10) = stance_current(ind);
                    sleepingShat(:, ind+10) = fHist(frameNum-1).targetList.S(:, tid);
                    %trackingHist(ind+10).hMeshU = trackingHist(ind).hMeshU;
                end
            end
            if(~isempty(trackingHist(ind).hMeshU))
                delete(trackingHist(ind).hPlotTrack);
                delete(trackingHist(ind).hPlotCentroid);
                delete(trackingHist(ind).hMeshU.UserData);
                delete(trackingHist(ind).hMeshU);
                delete(trackingHist(ind).hMeshG);
                delete(trackingHist(ind).hPlotAssociatedPoints);
                trackingHist(ind).hMeshU = [];
                trackingHist(ind).hMeshG = [];  
            end
                activeTracks(ind) = 0;
                overlap(ind) = 0;
                trackAge(ind) = 0;
                trackState(ind) = 0;
                height_vals(ind, :) = 0;
                stance_hist(ind, :) = 0;
                stance_current(ind) = 0;
        end
        iCount = find(sleepingTracks > 0);
        for n=1:length(iCount)
            ind = iCount(n);
            sleepingTracks(ind) = sleepingTracks(ind) + 1;
            if (sleepingTracks(ind) >= 200) % && sleepingStance(ind) ~= 1)
                delete(trackingHist(ind).hPlotTrack);
                delete(trackingHist(ind).hPlotCentroid);
                delete(trackingHist(ind).hMeshU.UserData);
                delete(trackingHist(ind).hMeshU);
                delete(trackingHist(ind).hMeshG);
                delete(trackingHist(ind).hPlotAssociatedPoints);
                trackingHist(ind).hMeshU = [];
                trackingHist(ind).hMeshG = [];           
                activeTracks(ind) = 0;
                sleepingTracks(ind) = 0;
                sleepingStance(ind) = 0;
            end
        end
        
        iReady = (activeTracks == 1);
        activeTracks(iReady) = 2;
        
        fHist(frameNum).done = 1000*toc(frameStart);
        
        string{1} = sprintf('Frame #: %d', targetFrameNum);
        string{2} = sprintf('Detection Points: %d', numOutputPoints);
        string{3} = sprintf('People Count: %d', peopleCountTotal);
%         s4 = 'Stance' + stance + ': %03d';
%         string{4} = sprintf(s4, p1height);
        for i=1:scene.numberOfTargetBoxes
            string{3+i} = sprintf('Box %d Count: %d', i, peopleCountInBox(i));
        end
        string{4+scene.numberOfTargetBoxes+1} = sprintf('Bytes Available: %d/%d', bytesAvailable, maxBytesAvailable);

        for n=1:length(hStatGlobal)
            set(hStatGlobal(n),'String',string{n});
        end
        
        
        for nBoxes = 1:scene.numberOfTargetBoxes    
            if(peopleCountInBox(nBoxes))
                set(hTargetBoxHandle(nBoxes), 'LineWidth', 14);
            else
                set(hTargetBoxHandle(nBoxes), 'LineWidth', 4);
            end
        end

        if(getappdata(hPbExit, 'exitKeyPressed') == 1)
            if(frameNumLogged > 10000)
                fHist = [fHist(frameNum+1:end) fHist(1:frameNum)];
            else
                fHist = fHist(1:frameNum);
            end
            
            save('fhistRT.mat','fHist');
            disp('Saving data and exiting');
            close_main()
            return;
        end
        
        frameNum = frameNum + 1;
        frameNumLogged = frameNumLogged + 1;      
        if(frameNum > 10000)
            frameNum = 1;
        end
                
        point3D = [posAll; xyz(2,:)];
        
        if(bytesAvailable > 32000)
            runningSlow  = 1;
        elseif(bytesAvailable < 1000)
            runningSlow = 0;
        end
        
        if(runningSlow)
            % Don't pause, we are slow
        else
            pause(0.01);
        end
    end
    
    if(targetFrameNum)
        lostSyncTime = tic;
        bytesAvailable = get(hDataSerialPort,'BytesAvailable');
        disp(['Lost sync at frame ', num2str(targetFrameNum),'(', num2str(frameNum), '), Reason: ', reason, ', ', num2str(bytesAvailable), ' bytes in Rx buffer']);
    else
        errordlg('Port sync error: Please close and restart program');
    end
%{
    % To catch up, we read and discard all uart data
    bytesAvailable = get(hDataSerialPort,'BytesAvailable');
    disp(bytesAvailable);
    [rxDataDebug, byteCountDebug] = fread(hDataSerialPort, bytesAvailable, 'uint8');
%}    
    while(lostSync)
        for n=1:8
            [rxByte, byteCount] = fread(hDataSerialPort, 1, 'uint8');
            if(rxByte ~= syncPatternUINT8(n))
                outOfSyncBytes = outOfSyncBytes + 1;
                break;
            end
        end
        if(n == 8)
            lostSync = 0;
            frameNum = frameNum + 1;
            if(frameNum > 10000)
                frameNum = 1;
            end
            
            [header, byteCount] = fread(hDataSerialPort, frameHeaderLengthInBytes - 8, 'uint8');
            rxHeader = [syncPatternUINT8'; header];
            byteCount = byteCount + 8;
            gotHeader = 1;
        end
    end
end

%% Helper functions

%Display Chirp parameters in table on screen
function h = displayChirpParams(Params, Position, hFig)

    dat =  {'Start Frequency (Ghz)', Params.profileCfg.startFreq;...
            'Slope (MHz/us)', Params.profileCfg.freqSlopeConst;...   
            'Samples per chirp', Params.profileCfg.numAdcSamples;...
            'Chirps per frame',  Params.dataPath.numChirpsPerFrame;...
            'Frame duration (ms)',  Params.frameCfg.framePeriodicity;...
            'Sampling rate (Msps)', Params.profileCfg.digOutSampleRate / 1000;...
            'Bandwidth (GHz)', Params.profileCfg.freqSlopeConst * Params.profileCfg.numAdcSamples /...
                               Params.profileCfg.digOutSampleRate;...
            'Range resolution (m)', Params.dataPath.rangeResolutionMeters;...
            'Velocity resolution (m/s)', Params.dataPath.dopplerResolutionMps;...
            'Number of Rx (MIMO)', Params.dataPath.numRxAnt; ...
            'Number of Tx (MIMO)', Params.dataPath.numTxAnt;};
    columnname =   {'Chirp Parameter (Units)      ', 'Value'};
    columnformat = {'char', 'numeric'};
    
    h = uitable('Parent',hFig,'Units','normalized', ...
            'Position', Position, ...
            'Data', dat,... 
            'ColumnName', columnname,...
            'ColumnFormat', columnformat,...
            'ColumnWidth', 'auto',...
            'RowName',[]);
end

function [P] = parseCfg(cliCfg)
    P=[];
    for k=1:length(cliCfg)
        C = strsplit(cliCfg{k});
        if strcmp(C{1},'channelCfg')
            P.channelCfg.txChannelEn = str2double(C{3});
            P.dataPath.numTxAzimAnt = bitand(bitshift(P.channelCfg.txChannelEn,0),1) +...
                                      bitand(bitshift(P.channelCfg.txChannelEn,-1),1);
            P.dataPath.numTxElevAnt = 0;
            P.channelCfg.rxChannelEn = str2double(C{2});
            P.dataPath.numRxAnt = bitand(bitshift(P.channelCfg.rxChannelEn,0),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-1),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-2),1) +...
                                  bitand(bitshift(P.channelCfg.rxChannelEn,-3),1);
            P.dataPath.numTxAnt = P.dataPath.numTxElevAnt + P.dataPath.numTxAzimAnt;
                                
        elseif strcmp(C{1},'dataFmt')
        elseif strcmp(C{1},'profileCfg')
            P.profileCfg.startFreq = str2double(C{3});
            P.profileCfg.idleTime =  str2double(C{4});
            P.profileCfg.rampEndTime = str2double(C{6});
            P.profileCfg.freqSlopeConst = str2double(C{9});
            P.profileCfg.numAdcSamples = str2double(C{11});
            P.profileCfg.digOutSampleRate = str2double(C{12}); %uints: ksps
        elseif strcmp(C{1},'chirpCfg')
        elseif strcmp(C{1},'frameCfg')
            P.frameCfg.chirpStartIdx = str2double(C{2});
            P.frameCfg.chirpEndIdx = str2double(C{3});
            P.frameCfg.numLoops = str2double(C{4});
            P.frameCfg.numFrames = str2double(C{5});
            P.frameCfg.framePeriodicity = str2double(C{6});
        elseif strcmp(C{1},'guiMonitor')
            P.guiMonitor.detectedObjects = str2double(C{2});
            P.guiMonitor.logMagRange = str2double(C{3});
            P.guiMonitor.rangeAzimuthHeatMap = str2double(C{4});
            P.guiMonitor.rangeDopplerHeatMap = str2double(C{5});
        end
    end
    P.dataPath.numChirpsPerFrame = (P.frameCfg.chirpEndIdx -...
                                            P.frameCfg.chirpStartIdx + 1) *...
                                            P.frameCfg.numLoops;
    P.dataPath.numDopplerBins = P.dataPath.numChirpsPerFrame / P.dataPath.numTxAnt;
    P.dataPath.numRangeBins = pow2roundup(P.profileCfg.numAdcSamples);
    P.dataPath.rangeResolutionMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                     (2 * P.profileCfg.freqSlopeConst * 1e12 * P.profileCfg.numAdcSamples);
    P.dataPath.rangeIdxToMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
                     (2 * P.profileCfg.freqSlopeConst * 1e12 * P.dataPath.numRangeBins);
    P.dataPath.dopplerResolutionMps = 3e8 / (2*P.profileCfg.startFreq*1e9 *...
                                        (P.profileCfg.idleTime + P.profileCfg.rampEndTime) *...
                                        1e-6 * P.dataPath.numDopplerBins * P.dataPath.numTxAnt);
end


function [] = dispError()
    disp('Serial Port Error!');
end

function exitPressFcn(hObject, ~)
    setappdata(hObject, 'exitKeyPressed', 1);
end

function resendPressFcn(hObject, ~)
    hSetup = hObject.UserData;
    hControlSerialPort = hSetup.hControlSerialPort;
    cliCfg = readCfg(hSetup.cfg.filename);
%     [Params cliCfg] = parseCfg(cliCfg, handles.angle); 
%     handles.params = Params;
%     guidata(hObject, handles);
    %Send CLI configuration to IWR16xx
    fprintf('Sending configuration from %s file to IWR6xxx ...\n', hSetup.cfg.filename);
    disp(hControlSerialPort);
    for k=1:length(cliCfg)
        fprintf(hControlSerialPort, cliCfg{k});
        fprintf('%s\n', cliCfg{k});
        echo = fgetl(hControlSerialPort); % Get an echo of a command
        done = fgetl(hControlSerialPort); % Get "Done" 
%         prompt = fread(hControlSerialPort, size(mmwDemoCliPrompt,2)); % Get the prompt back 
    end
end

function checkPlotTabs(hObject, eventData, hTabGroup)
    if(hObject.Value)
        % get children
        children = hTabGroup(3).Children;
        hTabGroup(3).UserData = children; %save children to restore
        
        % combine tab group
        for t=1:length(children)
            set(children(t),'Parent',hTabGroup(2));
        end
        
        % resize tab group
        hTabGroup(2).UserData = hTabGroup(2).Position; %save position to restore 
        hTabGroup(2).Position = [0.2 0 0.8 1];
        hTabGroup(3).Visible = 'off';
    else
        % restore children
        children = hTabGroup(3).UserData;
                
        % move tab group
        for t=1:length(children)
            set(children(t),'Parent',hTabGroup(3));
        end
        
        % resize tab group
        hTabGroup(2).Position = hTabGroup(2).UserData; 
        hTabGroup(3).Visible = 'on';
    end
        
end

function [sphandle] = configureDataSport(comPortNum, bufferSize)
    if ~isempty(instrfind('Type','serial'))
        disp('Serial port(s) already open. Re-initializing...');
        delete(instrfind('Type','serial'));  % delete open serial ports.
    end
    comPortString = ['COM' num2str(comPortNum)];
    sphandle = serial(comPortString,'BaudRate',921600);
    set(sphandle,'Terminator', '');
    set(sphandle,'InputBufferSize', bufferSize);
    set(sphandle,'Timeout',10);
    set(sphandle,'ErrorFcn',@dispError);
    fopen(sphandle);
end

function [sphandle] = configureControlPort(comPortNum)
    %if ~isempty(instrfind('Type','serial'))
    %    disp('Serial port(s) already open. Re-initializing...');
    %    delete(instrfind('Type','serial'));  % delete open serial ports.
    %end
    comPortString = ['COM' num2str(comPortNum)];
    sphandle = serial(comPortString,'BaudRate',115200);
    set(sphandle,'Parity','none')    
    set(sphandle,'Terminator','LF')        
    fopen(sphandle);
end

function config = readCfg(filename)
    config = cell(1,100);
    fid = fopen(filename, 'r');
    if fid == -1
        fprintf('File %s not found!\n', filename);
        return;
    else
        fprintf('Opening configuration file %s ...\n', filename);
    end
    tline = fgetl(fid);
    k=1;
    while ischar(tline)
        config{k} = tline;
        tline = fgetl(fid);
        k = k + 1;
    end
    config = config(1:k-1);
    fclose(fid);
end

function length = lengthFromStruct(S)
    fieldName = fieldnames(S);
    length = 0;
    for n = 1:numel(fieldName)
        [~, fieldLength] = S.(fieldName{n});
        length = length + fieldLength;
    end
end

function [R] = readToStruct(S, ByteArray)
    fieldName = fieldnames(S);
    offset = 0;
    for n = 1:numel(fieldName)
        [fieldType, fieldLength] = S.(fieldName{n});
        R.(fieldName{n}) = typecast(uint8(ByteArray(offset+1:offset+fieldLength)), fieldType);
        offset = offset + fieldLength;
    end
end
function CS = validateChecksum(header)
    h = typecast(uint8(header),'uint16');
    a = uint32(sum(h));
    b = uint16(sum(typecast(a,'uint16')));
    CS = uint16(bitcmp(b));
end

function [H] = computeH(~, s)
    posx = s(1); posy = s(2); velx = s(3); vely = s(4);
    range = sqrt(posx^2+posy^2);
    if posy == 0
        azimuth = pi/2;
    elseif posy > 0
        azimuth = atan(posx/posy);
    else
        azimuth = atan(posx/posy) + pi;
    end
    doppler = (posx*velx+posy*vely)/range;
    H = [range azimuth doppler]';
end

function [XX, YY, ZZ, v] = gatePlot3(~, G, C, A)
    %Extract the ellipsoid's axes lengths (a,b,c) and the rotation matrix (V) using singular value decomposition:
    [~,D,V] = svd(A/G);

    a = 1/sqrt(D(1,1));
    b = 1/sqrt(D(2,2));
    c = 1/sqrt(D(3,3));
    v = 4*pi*a*b*c/3;

    % generate ellipsoid at 0 origin
    [X,Y,Z] = ellipsoid(0,0,0,a,b,c);
    XX = zeros(size(X));
    YY = zeros(size(X));
    ZZ = zeros(size(X));
    for k = 1:length(X)
        for j = 1:length(X)
            point = [X(k,j) Y(k,j) Z(k,j)]';
            P = V * point;
            XX(k,j) = P(1)+C(1);
            YY(k,j) = P(2)+C(2);
            ZZ(k,j) = P(3)+C(3);
        end
    end
end

function [maxDim] = getDim(~, G, C, A)
    %Extract the ellipsoid's axes lengths (a,b,c) and the rotation matrix (V) using singular value decomposition:
    [~,D,V] = svd(A/G);

    a = 1/sqrt(D(1,1));
    b = 1/sqrt(D(2,2));
    c = 1/sqrt(D(3,3));
    
    maxDim = max([a,b]);

end

function [y] = pow2roundup (x)
    y = 1;
    while x > y
        y = y * 2;
    end
end

function h = circle(ax, x,y,r, LS)
d = r*2;
px = x-r;
py = y-r;
dim = [px py d d];
h = rectangle(ax, 'Position',dim,'Curvature',[1,1], 'LineWidth',3, 'LineStyle', LS);
daspect([1,1,1])
end

function h = filledCircle(ax, x,y,r, tColor)
d = r*2;
px = x-r;
py = y-r;
dim = [px py d d];
h = rectangle(ax, 'Position',dim,'Curvature',[1,1], 'LineWidth',3, 'FaceColor', tColor);
daspect([1,1,1])
end

function h = updateCenter(x,y,r,offset)
d = r*2;
px = x-r;
py = y-r;
h = [px py d d];
end


function close_main()
    %helpdlg('Saving and closing'); 
    open_port = instrfind('Type','serial','Status','open');
    for i=1:length(open_port)
        fclose(open_port(i));
        delete(open_port(i));
    end
    clear all 
    delete(findobj('Tag', 'mainFigure'));
    
end


function mypreview_fcn(obj,event,himage)
% Example update preview window function.

% Display image data.
himage.CData = fliplr(event.Data);
end

function [resInd] = getWidestFOV(resList)
maxR = 1;
resInd = 1;
    for i=1:length(resList)
        ss = strsplit(resList{i},'x');
        imWidth = str2num(ss{1});
        imHeight = str2num(ss{2});
        r = imWidth/imHeight;
        if (r>maxR)
            maxR = r;
            resInd = i;
        end
    end
end

function [height] = getHeight(tid, height_vals)
    avg_vals = height_vals(tid + 1, :) ~= 0;
    avg_vals = height_vals(tid + 1, avg_vals);
    height = mean(avg_vals);
end

function[yy] = maFilter(y, windowsize)
    b = (1/windowsize)*ones(1, windowsize);
    a = 1;
    yy = filter(b, a, y);
end

function [loc] = isInBox(S, box)
    x = S(1, 1);
    z = S(3, 1);
    
    if (box(1) < x && box(2) > x)
        if (box(3) < z && box(4) > z)
            loc = 1;
        else
            loc = 0;
        end
    else
        loc = 0;
    end

end

function [sleepingTracks, woke, im] =  checkFreeSleepers(n, sleepingTracks, sleepingShat, S)
    dim = 0.3;
    woke = 0;
    sleepers = find(sleepingTracks > 0);
    im = [];
    for m = 1:size(sleepers, 2)
        im = sleepers(m);
        if(hypot(S(1,n) - sleepingShat(1,im), S(3,n) - sleepingShat(3,im)) < 2*dim)
            sleepingTracks(im) = 1000;
            woke = 1;
            break;
        end                       
    end
    if (isempty(im))
        im = -1;
    end
end
