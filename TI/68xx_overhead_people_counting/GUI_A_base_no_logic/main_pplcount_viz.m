clear, clc

%% SETUP SELECTIONS

% 1. sceneRun needs to be specied.
% Default is to use GUI to setup scene
% To programmatically setup the scene follow the example in Prgm_2box.
 
% Scene Run choice: {'GUI_Setup' | 'Prgm_2box' | 'Prgm_MaxFOV','My_Scene'};
% sceneRun = 'GUI_Setup';
sceneRun = 'Prgm_2box';
orientation = 'sideMount';
% orientation = 'Overhead';
tColor = [0.5 1 0.5];
sensorHeight = 1.97;
sensorTilt = -0;
sensorTilt = deg2rad(sensorTilt);
maxHeight = 2;

% 2. Specify COM ports if programmatically setting up scene
if (~strcmp(sceneRun,'GUI_Setup'))
    %%%%% EDIT COM PORTS %%%%%%
    controlSerialPort = 4;
    dataSerialPort = 3;
    loadCfg = 1;
end


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
    
    %Read Chirp Configuration file
    configurationFileName = 'mmw_pplcount_demo_default.cfg';   
    cliCfg = readCfg(configurationFileName);
    Params = parseCfg(cliCfg);
    
    cfg.filename = configurationFileName;
    
    hSetup.hControlSerialPort = controlSerialPort;
    hSetup.cfg = cfg;
    
    % Room Wall dimensions [m]
    % Measured relative to radar
    wall.left = -6; % signed: - required
    wall.right = 6;
    wall.front = 6;
    wall.back = -6; % signed: - required
    
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
    configurationFileName = 'mmw_pcdemo_default_standard_Operation.cfg';   
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


figureTitles = {'Statistics', 'Point Cloud', 'Gating and Association', 'Chirp Configuration', 'Visualizer Options & Control'}; %, 'Heat Map', 'Feature Ext'};
figureGroup = [1, 2, 3, 1, 1];
numFigures = size(figureTitles, 2);
hFigure = zeros(1,numFigures);

hTabGroup(1) = uitabgroup(figHandle, 'Position', [0.0 0 0.2 1]);

if((wall.right-wall.left) > (wall.front - wall.back))
   hTabGroup(2) = uitabgroup(figHandle, 'Position', [0.2 0.5 0.8 0.5]);
   hTabGroup(3) = uitabgroup(figHandle, 'Position', [0.2 0.0 0.8 0.5]); 
else
   hTabGroup(2) = uitabgroup(figHandle, 'Position', [0.2 0 0.4 1]);
   hTabGroup(3) = uitabgroup(figHandle, 'Position', [0.6 0 0.4 1]); 
end

for iFig = 1:5
    hFigure(iFig) = uitab(hTabGroup(figureGroup(iFig)), 'Title', figureTitles{iFig});

    if(strcmp(figureTitles{iFig},'Point Cloud'))
        trackingAx = axes('parent', hFigure(iFig));

        %plot(trackingAx, sensor.rangeMin*sin(sensor.angles+scene.azimuthTilt), sensor.rangeMin*cos(sensor.angles+scene.azimuthTilt), '-k');  hold on;
        %plot(trackingAx, [0 sensor.rangeMax*sin(sensor.angles+scene.azimuthTilt) 0],[0 sensor.rangeMax*cos(sensor.angles+scene.azimuthTilt) 0], '-k');
  
        title(figureTitles{iFig},'FontUnits','Normalized', 'FontSize',0.05);
        axis equal;
        axis(scene.maxPos);
        camroll(180)
        
        % draw wall box
        rectangle(trackingAx, 'Position', scene.areaBox, 'EdgeColor','k', 'LineStyle', '-', 'LineWidth', 2);
        
        % draw target box
        for nBoxes = 1:scene.numberOfTargetBoxes
            hTargetBoxHandle(nBoxes)= rectangle('Parent', trackingAx, 'Position', scene.targetBox(nBoxes,:), 'EdgeColor', colors(nBoxes), 'LineWidth', 4);
        end
        
        
        grid on;
        hold on;
        grid minor;

    end
    
    if(strcmp(figureTitles{iFig},'Gating and Association'))
        gatingAx = axes('parent', hFigure(iFig));

        %plot(gatingAx, sensor.rangeMin*sin(sensor.angles+scene.azimuthTilt), sensor.rangeMin*cos(sensor.angles+scene.azimuthTilt), '-k');  hold on;
        %plot(gatingAx, [0 sensor.rangeMax*sin(sensor.angles+scene.azimuthTilt) 0],[0 sensor.rangeMax*cos(sensor.angles+scene.azimuthTilt) 0], '-k');
        cax = [1 2];
        title(figureTitles{iFig},'FontUnits','Normalized', 'FontSize',0.05);
        axis equal;
        axis(scene.maxPos);        
        cbar = colorbar('Ticks', [0.9, 1.3, 1.7], 'TickLabels', {'Laying', 'Sitting', 'Standing'});
        caxis([0.7 2]);
%       
        %colormap(cool(64))
       
        camroll(180)
        
        % draw wall box
        rectangle(gatingAx, 'Position', scene.areaBox, 'EdgeColor','k', 'LineStyle', '-', 'LineWidth', 2);
        
        % draw target box
        for nBoxes = 1:scene.numberOfTargetBoxes
            hTargetBoxHandle(nBoxes)= rectangle('Parent', gatingAx, 'Position', scene.targetBox(nBoxes,:), 'EdgeColor', colors(nBoxes), 'LineWidth', 4);
        end
        
        grid on;
        hold on;
        grid minor;

    end
    
    if(strcmp(figureTitles{iFig},'Chirp Configuration'))
        %axes('parent', hFigure(iFig))
        tablePosition = [0.1 0.45 0.8 0.5];
        displayChirpParams(Params, tablePosition, hFigure(iFig));
    end
    
    if(strcmp(figureTitles{iFig},'Statistics'))     
        axes('parent', hFigure(iFig))
        hStatGlobal(1) = text(0, 0.9, 'Frame # 0', 'FontSize',12, 'Visible', 'on');
        hStatGlobal(2) = text(0, 0.8, 'Detection Points: 0','FontSize',12, 'Visible', 'on');
        hStatGlobal(3) = text(0, 0.6, 'People Count:  0','FontSize',24);
        hStatGlobal(4) = text(0, 0.4, 'Target Height:', 'FontSize', 18); 
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
            'Position', [10 100 100 40], 'UserData', hSetup, 'Callback', @resendPressFcn);
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
    'accZ',             {'float', 4});%, ...
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

height_vals = zeros(20, 20);
height_tid = zeros(20,1);
%num_meas = zeros(20, 1);

update = 0;
%lostSyncTime = 0;
%% Main
fhistNum = 0;
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
                            xyz = zeros(3, numInputPoints);
                            %fix elev based on sensor tilt when side
                            %mounted
                            if (strcmp(orientation, 'sideMount'))
                                pointCloud(3,:) = pointCloud(3,:)+sensorTilt;
                                xyz(3, :) = (pointCloud(1,:).*sin(pointCloud(3,:))) + sensorHeight;
                            else
                                xyz(3, :) = (pointCloud(1,:).*sin(pointCloud(3,:)));
                            end
                            r = pointCloud(1,:).*cos(pointCloud(3,:));
                            xyz(2, :) = r.*cos(pointCloud(2,:));
                            xyz(1, :) = r.*sin(pointCloud(2,:));
                            if (~strcmp(orientation, 'sideMount'))
                                posAll = [xyz(1,:); xyz(3,:)];
                            else
                                posAll = [xyz(1,:); xyz(2,:)];
                            end
                            snrAll = pointCloud(5,:);
                            

                            % Remove out of Range, Behind the Walls, out of FOV points
                            inRangeInd = (pointCloud(1,:) > 1) & (pointCloud(1,:) < 6) & ...
                                (pointCloud(2,:) > -50*pi/180) &  (pointCloud(2,:) < 50*pi/180) & ...
                                (posAll(1,:) > scene.areaBox(1)) & (posAll(1,:) < (scene.areaBox(1) + scene.areaBox(3))) & ...
                                (posAll(2,:) > scene.areaBox(2)) & (posAll(2,:) < (scene.areaBox(2) + scene.areaBox(4)));
                            pointCloudInRange = pointCloud(:,inRangeInd);
                            posInRange = posAll(:,:);
%{
                            % Clutter removal
                            staticInd = (pointCloud(3,:) == 0);        
                            clutterInd = ismember(pointCloud(1:2,:)', clutterPoints', 'rows');
                            clutterInd = clutterInd' & staticInd;
                            clutterPoints = pointCloud(1:2,staticInd);
                            pointCloud = pointCloud(1:3,~clutterInd);
%}
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
                            TID(n)  = typecast(uint8(rxData(offset+1:offset+4)),'uint32');
                            if (floor(TID(n)) ~= TID(n) || TID(n) > 20)
                                lostSync = 1;
                                break;
                            end
                            %1x4=4bytes
                            S(:,n)  = typecast(uint8(rxData(offset+5:offset+40)),'single');     %9x4=36bytes
%                             EC(:,n) = typecast(uint8(rxData(offset+41:offset+104)),'single');    %16x4=64bytes
%                             G(n)    = typecast(uint8(rxData(offset+105:offset+108)),'single');    %1x4=4bytes
                            offset = offset + 40;
%                             if (TID < 21)
%                                 height_vals(TID(n) + 1, mod(frameNum, 10) + 1) = 3 - S(2, n);
%                             end
%                             he = getHeight(TID(n), height_vals);
% %                             fHist(frameNum).height = [fHist(frameNum).height he];
%                             fHist(frameNum).height = [fHist(frameNum).height he];

                        end
                        
                    case 8
                        % Target Index TLV
                        numIndices = valueLength/indexLengthInBytes;
                        mIndex = typecast(uint8(rxData(offset+1:offset+numIndices)),'uint8');
                        offset = offset + valueLength;
                        tind = unique(mIndex);
                        for i=1:size(tind)
                            if (tind(i) < 30) 
                                h_ind = mIndex == tind(i);
                                h_ind = point3D(3, h_ind);
                                %filtering used if overhead
%                                 d_ind = h_ind(1, :) > 0.5 & h_ind(1, :) < 3;
%                                 h_ind = h_ind(1, d_ind);
                                height_vals(tind(i) + 1, mod(frameNum, 20) + 1) = max(h_ind);
                                height_tid(tind(i)+1,1) = getHeight(tind(i), height_vals);
                                fHist(frameNum).height = [fHist(frameNum).height height_tid(tind(i)+1,1)];
                                if (i == 1)
                                    p1height = height_tid(tind(i)+1);
                                end
                            end
                        end
%                         if(p1>=0 && p1 <=20 && size(mIndex,1) == size(point3D, 2))
%                             p1ind = mIndex == p1;
%                             p1vals = point3D(3, p1ind);
%                             height = min(p1vals);
%                             p1height = 3 - height;
%                             if (p1height >= 1.7 && p1height < 3)
%                                 stance = "Standing";
%                             elseif (p1height >= 1.2 && p1height < 1.7)
%                                 stance = "Sitting";
%                             elseif (p1height < 1.2 && p1height > 0)
%                                 stance = "Laying";
%                             else
%                                 stance = "Unknown";
%                             end
%                         else
%                             p1height = -1;
%                         end
                        
                end
            end
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

        if(size(posAll,2))
            % Plot all points
            if(sum(snrAll) >= 0)
                if(~optimize)
                    hPlotCloudHandleAll = scatter(trackingAx, posAll(1,:), posAll(2,:),'.k','SizeData',snrAll*10);
                else
                    hPlotCloudHandleAll = plot(trackingAx, posAll(1,:), posAll(2,:),'.k');
                end           
            end
            % Cross out out-of-Range
            if(~optimize)
                hPlotCloudHandleOutRange = plot(trackingAx, posAll(1,~inRangeInd), posAll(2,~inRangeInd), 'xr');
            end
        end
        
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
        if nnz(isnan(EC))
            reason = 'Error: EC contains NaNs';
            lostSync = 1;
            break;
        end
        
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
            if(tid > maxNumTracks)
                reason = 'Error: TID is wrong';
                lostSync = 1;
                break;
            end
            cmap = colormap(cbar);
            tColor = round((height_tid(tid)/maxHeight) * size(cmap,1));
            if (tColor > size(cmap, 1))
                tColor = size(cmap, 1);
            end
            if (tColor < 1)
                tColor = 1;
            end
% %                 disp(tColor);
            tColor = cmap(tColor, :);
            if( (size(mIndex,1) > 0) && (size(mIndex,1) == size(point3D,2)) )
%                 tColor = colors(mod(tid,length(colors))+1);
%                   tColor = colors(1);
%                 cmap = colormap(cbar);
%                 tColor = round(((fHist(frameNum).height(n) - cax(1))/(cax(2) - cax(1))) * size(cmap,1));
%                 if (tColor > size(cmap, 1))
%                     tColor = size(cmap, 1);
%                 end
%                 if (tColor < 1)
%                     tColor = 1;
%                 end
% %                 disp(tColor);
%                 tColor = cmap(tColor, :);
                ind = (mIndex == tid);
                valid = point3D(:, ind);
                dim = 0.3;
                if (strcmp(orientation, 'sideMount'))
                    ind = (hypot(S(1,n) - valid(1,:), S(2,n) - valid(2, :)) <= dim);
                else
                    ind = (hypot(S(1,n) - valid(1,:), S(3,n) - valid(2, :)) <= dim);
                end
                if nnz(ind)
                    %trackingHist(tid).hPlotAssociatedPoints = plot(gatingAx, point3D(1,ind), point3D(2,ind), 'o', 'color', tColor)
                    if (isempty(trackingHist(tid).hPlotAssociatedPoints) || ~ishandle(trackingHist(tid).hPlotAssociatedPoints))
%                         trackingHist(tid).hPlotAssociatedPoints = plot(gatingAx, point3D(1,ind), point3D(2,ind), '.', 'MarkerSize', 10, 'color', tColor);
                        trackingHist(tid).hPlotAssociatedPoints = plot(gatingAx, valid(1,ind), valid(2,ind), '.', 'MarkerSize', 10, 'color', tColor);
                    else
                        if ishandle(trackingHist(tid).hPlotAssociatedPoints)
%                             set(trackingHist(tid).hPlotAssociatedPoints, 'XData', point3D(1,ind),'YData', point3D(2,ind), 'color', tColor);
                            set(trackingHist(tid).hPlotAssociatedPoints, 'XData', valid(1,ind),'YData', valid(2,ind), 'color', tColor);
                        end
                    end
                end
            end
               
            %g = G(n);
            centroid = computeH(1, S(:,n));
%             ec = reshape(EC(:,n),4,4);
            if(1)
%                 dim = getDim(gatingAx, 1, centroid, ec);
                dim = 0.3;
                if isempty(trackingHist(tid).hMeshU)
                    if(strcmp(orientation, 'sideMount'))
                        trackingHist(tid).hMeshU = circle(gatingAx,S(1,n), S(2,n),dim);
                    else
                        trackingHist(tid).hMeshU = circle(gatingAx,S(1,n), S(3,n),dim);
                    end
                    if(~optimize)
                        trackingHist(tid).hMeshU.EdgeColor = [0.5 0.5 0.5];
                    else
                        trackingHist(tid).hMeshU.EdgeColor = tColor;
                    end
                    trackingHist(tid).hMeshU.FaceColor = 'none';
                else
                    %set(trackingHist(tid).hMeshU, 'XData', xU.*sin(yU),'YData',xU.*cos(yU), 'ZData', zU);
                    if(strcmp(orientation, 'sideMount'))
                        trackingHist(tid).hMeshU.Position = updateCenter(S(1,n), S(2,n),dim);
                    else
                        trackingHist(tid).hMeshU.Position = updateCenter(S(1,n), S(3,n),dim);
                    end
                    trackingHist(tid).hMeshU.EdgeColor = tColor;
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
                        set(trackingHist(tid).hMeshG, 'XData', xG.*sin(yG),'YData',xG.*cos(yG), 'ZData', zG);
                    end
                end
            end
            
            if(activeTracks(tid) == 0)
                activeTracks(tid) = 1;
                trackingHist(tid).tid = TID(n);
                trackingHist(tid).allocationTime = targetFrameNum;
                trackingHist(tid).tick = 1;
                trackingHist(tid).posIndex = 1;
                trackingHist(tid).histIndex = 1;
                trackingHist(tid).sHat(1,:) = S(:,n);
                if (~strcmp(orientation, 'sideMount'))
                    trackingHist(tid).pos(1,:) = S(1:3,n);               
    %                 trackingHist(tid).hPlotTrack = plot(trackingAx, S(1,n), S(3,n), '-', 'LineWidth', 2, 'color', colors(mod(tid,length(colors))+1));
    %                 trackingHist(tid).hPlotCentroid = plot(trackingAx, S(1,n), S(3,n), 'o', 'color', colors(mod(tid,length(colors))+1));
                    trackingHist(tid).hPlotTrack = plot(trackingAx, S(1,n), S(3,n), '-', 'LineWidth', 2, 'color', colors(1));
                    trackingHist(tid).hPlotCentroid = plot(trackingAx, S(1,n), S(3,n), 'o', 'color', colors(1));
                else
                    trackingHist(tid).pos(1,:) = S(1:3,n);               
    %                 trackingHist(tid).hPlotTrack = plot(trackingAx, S(1,n), S(3,n), '-', 'LineWidth', 2, 'color', colors(mod(tid,length(colors))+1));
    %                 trackingHist(tid).hPlotCentroid = plot(trackingAx, S(1,n), S(3,n), 'o', 'color', colors(mod(tid,length(colors))+1));
                    trackingHist(tid).hPlotTrack = plot(trackingAx, S(1,n), S(2,n), '-', 'LineWidth', 2, 'color', colors(1));
                    trackingHist(tid).hPlotCentroid = plot(trackingAx, S(1,n), S(2,n), 'o', 'color', colors(1));
                end
            else
                activeTracks(tid) = 1;                
                trackingHist(tid).tick = trackingHist(tid).tick + 1;
                
                trackingHist(tid).histIndex = trackingHist(tid).histIndex + 1;
                if(trackingHist(tid).histIndex > 1000)
                    trackingHist(tid).histIndex = 1;
                end
                trackingHist(tid).sHat(trackingHist(tid).histIndex,:) = S(:,n);

                trackingHist(tid).posIndex = trackingHist(tid).posIndex + 1;
                if(trackingHist(tid).posIndex > 100)
                    trackingHist(tid).posIndex = 1;
                end
                trackingHist(tid).pos(trackingHist(tid).posIndex,:) = S(1:3,n);

                if(trackingHist(tid).tick > 100)
                    if (strcmp(orientation, 'sideMount'))
                         set(trackingHist(tid).hPlotTrack, 'XData', [trackingHist(tid).pos(trackingHist(tid).posIndex+1:end,1); trackingHist(tid).pos(1:trackingHist(tid).posIndex,1)], ...
                        'YData',[trackingHist(tid).pos(trackingHist(tid).posIndex+1:end,2); trackingHist(tid).pos(1:trackingHist(tid).posIndex,2)]);
                    else
                        set(trackingHist(tid).hPlotTrack, 'XData', [trackingHist(tid).pos(trackingHist(tid).posIndex+1:end,1); trackingHist(tid).pos(1:trackingHist(tid).posIndex,1)], ...
                        'YData',[trackingHist(tid).pos(trackingHist(tid).posIndex+1:end,3); trackingHist(tid).pos(1:trackingHist(tid).posIndex,3)]);
                    end
                else
                    if (strcmp(orientation, 'sideMount'))
                        set(trackingHist(tid).hPlotTrack, 'XData', trackingHist(tid).pos(1:trackingHist(tid).posIndex,1), ...
                        'YData',trackingHist(tid).pos(1:trackingHist(tid).posIndex,2));
                    else
                        set(trackingHist(tid).hPlotTrack, 'XData', trackingHist(tid).pos(1:trackingHist(tid).posIndex,1), ...
                            'YData',trackingHist(tid).pos(1:trackingHist(tid).posIndex,3));
                    end
                end              
                if (strcmp(orientation, 'sideMount'))
                    set(trackingHist(tid).hPlotCentroid,'XData',S(1,n),'YData',S(2,n));
                else
                    set(trackingHist(tid).hPlotCentroid,'XData',S(1,n),'YData',S(3,n));
                end
                
                for nBoxes = 1:scene.numberOfTargetBoxes 
                    if( (S(1,n) > scene.targetBox(nBoxes,1)) && (S(1,n) < (scene.targetBox(nBoxes,1) + scene.targetBox(nBoxes,3))) && ...
                            (S(3,n) > scene.targetBox(nBoxes,2)) && (S(3,n) < (scene.targetBox(nBoxes,2)+scene.targetBox(nBoxes,4))) )
                        peopleCountInBox(nBoxes) = peopleCountInBox(nBoxes) +1;
                    end  
                end               
            end
        end
        
        iDelete = find(activeTracks == 2);
        for n=1:length(iDelete)
            ind = iDelete(n);
            delete(trackingHist(ind).hPlotTrack);
            delete(trackingHist(ind).hPlotCentroid);
            delete(trackingHist(ind).hMeshU.UserData);
            delete(trackingHist(ind).hMeshU);
            delete(trackingHist(ind).hMeshG);
            delete(trackingHist(ind).hPlotAssociatedPoints);
            trackingHist(ind).hMeshU = [];
            trackingHist(ind).hMeshG = [];           
            activeTracks(ind) = 0;
        end
        
        iReady = (activeTracks == 1);
        activeTracks(iReady) = 2;
        
        fHist(frameNum).done = 1000*toc(frameStart);
        
        string{1} = sprintf('Frame #: %d', targetFrameNum);
        string{2} = sprintf('Detection Points: %d', numOutputPoints);
        string{3} = sprintf('People Count: %d', peopleCountTotal);
        s4 = 'Target Height: %03d';
        string{4} = sprintf(s4, p1height);
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
        if (mod(frameNumLogged, 1200) == 0)
            frameNum = 1;
            save_name = ['fhistRT' num2str(fhistNum)];
            save(save_name, 'fHist');
            disp('Saving data');
            fhistNum = fhistNum + 1;
        end
        frameNum = frameNum + 1;
        frameNumLogged = frameNumLogged + 1;      
        if(frameNum > 10000)
            frameNum = 1;
        end
        if (strcmp(orientation, 'sideMount'))        
            point3D = [posAll; xyz(3,:)];
        else
            point3D = [posAll; xyz(2,:)];
        end
        
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

function h = circle(ax, x,y,r)
d = r*2;
px = x-r;
py = y-r;
dim = [px py d d];
h = rectangle(ax, 'Position',dim,'Curvature',[1,1], 'LineWidth',3);
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

% function [P, cliCfg] = parseCfg(cliCfg, azimuthTilt)
%     P=[];
%     for k=1:length(cliCfg)
%         C = strsplit(cliCfg{k});
%         if strcmp(C{1},'channelCfg')
%             P.channelCfg.txChannelEn = str2double(C{3});
%             P.dataPath.numTxAzimAnt = bitand(bitshift(P.channelCfg.txChannelEn,0),1) +...
%                                       bitand(bitshift(P.channelCfg.txChannelEn,-1),1);
%             P.dataPath.numTxElevAnt = 0;
%             P.channelCfg.rxChannelEn = str2double(C{2});
%             P.dataPath.numRxAnt = bitand(bitshift(P.channelCfg.rxChannelEn,0),1) +...
%                                   bitand(bitshift(P.channelCfg.rxChannelEn,-1),1) +...
%                                   bitand(bitshift(P.channelCfg.rxChannelEn,-2),1) +...
%                                   bitand(bitshift(P.channelCfg.rxChannelEn,-3),1);
%             P.dataPath.numTxAnt = P.dataPath.numTxElevAnt + P.dataPath.numTxAzimAnt;
%                                 
%         elseif strcmp(C{1},'trackingCfg')
%             % error check for azimuth tilt
%             % if cfg tilt is not same as GUI specified tilt; GUI specified
%             % tilt is used instead
%             if((str2num(C{8})-90)*pi/180 ~= azimuthTilt)
%                 temp = cliCfg{k};
%                 temp(end+1-length(C{8}):end) = '';
%                 ang = num2str(90-azimuthTilt);
%                 temp = [temp ang];
%                 cliCfg{k} = temp;
%                 fprintf('trackingCfg specifies %d.\n', (str2num(C{8})-90)*pi/180 );
%                 fprintf('GUI specifies %d. %s will be used for azimuth in cfg.\n',azimuthTilt, ang);
%             end
%         elseif strcmp(C{1},'profileCfg')
%             P.profileCfg.startFreq = str2double(C{3});
%             P.profileCfg.idleTime =  str2double(C{4});
%             P.profileCfg.rampEndTime = str2double(C{6});
%             P.profileCfg.freqSlopeConst = str2double(C{9});
%             P.profileCfg.numAdcSamples = str2double(C{11});
%             P.profileCfg.digOutSampleRate = str2double(C{12}); %uints: ksps
%         elseif strcmp(C{1},'chirpCfg')
%         elseif strcmp(C{1},'frameCfg')
%             P.frameCfg.chirpStartIdx = str2double(C{2});
%             P.frameCfg.chirpEndIdx = str2double(C{3});
%             P.frameCfg.numLoops = str2double(C{4});
%             P.frameCfg.numFrames = str2double(C{5});
%             P.frameCfg.framePeriodicity = str2double(C{6});
%         elseif strcmp(C{1},'guiMonitor')
%             P.guiMonitor.detectedObjects = str2double(C{2});
%             P.guiMonitor.logMagRange = str2double(C{3});
%             P.guiMonitor.rangeAzimuthHeatMap = str2double(C{4});
%             P.guiMonitor.rangeDopplerHeatMap = str2double(C{5});
%         end
%     end
%     P.dataPath.numChirpsPerFrame = (P.frameCfg.chirpEndIdx -...
%                                             P.frameCfg.chirpStartIdx + 1) *...
%                                             P.frameCfg.numLoops;
%     P.dataPath.numDopplerBins = P.dataPath.numChirpsPerFrame / P.dataPath.numTxAnt;
%     P.dataPath.numRangeBins = pow2roundup(P.profileCfg.numAdcSamples);
%     P.dataPath.rangeResolutionMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
%                      (2 * P.profileCfg.freqSlopeConst * 1e12 * P.profileCfg.numAdcSamples);
%     P.dataPath.rangeIdxToMeters = 3e8 * P.profileCfg.digOutSampleRate * 1e3 /...
%                      (2 * P.profileCfg.freqSlopeConst * 1e12 * P.dataPath.numRangeBins);
%     P.dataPath.dopplerResolutionMps = 3e8 / (2*P.profileCfg.startFreq*1e9 *...
%                                         (P.profileCfg.idleTime + P.profileCfg.rampEndTime) *...
%                                         1e-6 * P.dataPath.numDopplerBins * P.dataPath.numTxAnt);
% end