
clear;
clf;

filenum = '014';
% DPW3:014  DPW2:011  DPW1:010  DPL2:002  DPL1:001  LF:006
START_SAMPLE = 3800;
% DPW3:3800 DPW2:4880 DPW1:2652 DPL2:1311 DPL1:2300 LF:1870
END_SAMPLE = 17450;
% DPW3:17450 DPW2:10500 DPW1:8720 DPL2:4284 DPL1:13425 LF:5384
infofile = strcat('INF', filenum, '.TXT');
datafile = strcat('LOG', filenum, '.BIN');

% map from datatype to length in bytes
dataSizes.('float') = 4;
dataSizes.('ulong') = 4;
dataSizes.('int') = 4;
dataSizes.('int32') = 4;
dataSizes.('uint8') = 1;
dataSizes.('uint16') = 2;
dataSizes.('char') = 1;

% read from info file to get log file structure
fileID = fopen(infofile);
items = textscan(fileID,'%s','Delimiter',',','EndOfLine','\r\n');
fclose(fileID);
[ncols,~] = size(items{1});
ncols = ncols/2;
varNames = items{1}(1:ncols)';
varTypes = items{1}(ncols+1:end)';
varLengths = zeros(size(varTypes));
colLength = 256;
for i = 1:numel(varTypes)
    varLengths(i) = dataSizes.(varTypes{i});
end

R = cell(1,numel(varNames));

% read column-by-column from datafile
fid = fopen(datafile,'rb');
for i=1:numel(varTypes)
    %# seek to the first field of the first record
    fseek(fid, sum(varLengths(1:i-1)), 'bof');
    
    %# % read column with specified format, skipping required number of bytes
    R{i} = fread(fid, Inf, ['*' varTypes{i}], colLength-varLengths(i));
    eval(strcat(varNames{i},'=','R{',num2str(i),'};'));
end
fclose(fid);

length(accelX)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        Useful Constants                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SERIES_RESISTOR = .125; %[Ohms]
SURFACE_PRESSURE = 224; %224 at Dana Point
DEPTH_OFFSET = .3; %[m]
DATA_LENGTH = END_SAMPLE-START_SAMPLE+1;
ENCODER_LENGTH_OFFSET = -.1; %[m]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           Convert 10 Bit Data Into Useful Units               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%IMU data
accelX = double(accelX(START_SAMPLE:END_SAMPLE));
accelY = double(accelY(START_SAMPLE:END_SAMPLE));
accelZ = double(accelZ(START_SAMPLE:END_SAMPLE));
headingIMU = double(headingIMU(START_SAMPLE:END_SAMPLE));
pitchIMU = double(pitchIMU(START_SAMPLE:END_SAMPLE));
rollIMU = double(rollIMU(START_SAMPLE:END_SAMPLE));
%GPS data
lat = double(lat(START_SAMPLE:END_SAMPLE));
lon = double(lon(START_SAMPLE:END_SAMPLE));
nsats = uint8(nsats(START_SAMPLE:END_SAMPLE));
%Computed Values
hdop = double(hdop(START_SAMPLE:END_SAMPLE));
heading = double(heading(START_SAMPLE:END_SAMPLE));
left = double(left(START_SAMPLE:END_SAMPLE));
right = double(right(START_SAMPLE:END_SAMPLE));
x = double(x(START_SAMPLE:END_SAMPLE));
y = double(y(START_SAMPLE:END_SAMPLE));
%Custom Sensors
Battery_Current = double(Battery_Current(START_SAMPLE:END_SAMPLE));
Battery_Voltage = double(Battery_Voltage(START_SAMPLE:END_SAMPLE));
Button_1 = uint8(Button_1(START_SAMPLE:END_SAMPLE));
Temperature_1 = double(Temperature_1(START_SAMPLE:END_SAMPLE));
Pressure = double(Pressure(START_SAMPLE:END_SAMPLE));
Encoder = double(Encoder(START_SAMPLE:END_SAMPLE));


% calibration relationships in the form y = a*x + b
% where:
% y = measurement in useful units
% x = measurement of 10 bit resolution from teensy ADC

% (1) Current Draw [A]
a1 = .0031644; % fit line slope
b1 = -.011786; % fit line intercept
Battery_Current = a1.*Battery_Current + b1; % [A]
% (2) Battery Voltage [V]
a2 = .012926; % fit line slope
b2 = -.040916; % fit line intercept
Battery_Voltage = a2.*Battery_Voltage + b2; % [V]
Battery_Voltage_Adjusted = Battery_Voltage + SERIES_RESISTOR.*Battery_Current; % [V]
% (3) Temperature Sensor #1 [C]
a3 = .014568; % fit line slope
b3 = 10.1818; % fit line intercept
Temperature_1 = a3.*Temperature_1 + b3; % [C]
% (4) Pressure Sensor to Depth [m]
a4 = .006; % fit line slope
b4 = -a4*SURFACE_PRESSURE + DEPTH_OFFSET; % fit line intercept
Depth = a4.*Pressure + b4; % [m]
% (5) Button
% (6) Encoder Stuff [m]
a6 = -.00002;
b6 = .0246;
c6 = .0122;
Length = a6*(Encoder).^2 + b6*(Encoder) + c6;
% (7) Number of Satellites Found
% (8) Heading
% (9) HeadingIMU
% (10) Acceleration [m/s^2]
% (11) Position [m]
% (12) Depth Map
% (13) Temperature Map
% (14) Encoder Angle
Theta = acos((abs(Depth-DEPTH_OFFSET))./abs(Length));
% (15) Encoder Error
Position_Error = sin(Theta).*Length;

% Filter some of the noisy signals
numAvgs = 30;
windowSize = 10;
%accelX = RunningAvg(accelX,numAvgs);
%accelY = RunningAvg(accelY,numAvgs);
%accelZ = RunningAvg(accelZ,numAvgs);
%headingIMU = RunningAvg(headingIMU,numAvgs);
%pitchIMU = RunningAvg(pitchIMU,numAvgs);
%rollIMU = RunningAvg(rollIMU,numAvgs);
%heading = RunningAvg(heading,numAvgs);
%Depth = RunningAvg(Depth,numAvgs);
Temperature_1 = MedianGuy(Temperature_1,windowSize);
Battery_Current = RunningAvg(Battery_Current,numAvgs);
Battery_Voltage = RunningAvg(Battery_Voltage,numAvgs);
Battery_Voltage_Adjusted = RunningAvg(Battery_Voltage_Adjusted,numAvgs);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     Print Relevant Stuff                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

time = linspace(0,DATA_LENGTH/10,DATA_LENGTH);% Temporary time vector

figure(1) % Current
plot(time,Battery_Current,'b','LineWidth',1);
ah = gca;
title('Current Draw From Battery Over Time');
xlabel('Time [s]');
ylabel('Current Draw [A]');
set(ah,'XLim',[1,time(end)]);
%set(ah,'YLim',[0,3]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
grid on

figure(2) % Voltage
plot(time,Battery_Voltage,'b','LineWidth',1);
hold on
plot(time,Battery_Voltage_Adjusted,'r','LineWidth',1);
ah = gca;
title('Battery Voltage Over Time');
xlabel('Time [s]');
ylabel('Battery Voltage [V]');
set(ah,'XLim',[1,time(end)]);
%set(ah,'YLim',[0,3]);
set(ah,'FontSize',14);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',2);
legend('V_b_a_t','V_b_a_t Adjusted')
grid on

figure(3) % Temperature
plot(time,Temperature_1,'b','LineWidth',1);
ah = gca;
title('Temperature Over Time');
xlabel('Time [s]');
ylabel('Temperature [C]');
set(ah,'XLim',[1,time(end)]);
%set(ah,'YLim',[0,3]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
grid on

figure(4) % Depth
plot(time,Depth,'b','LineWidth',1);
ah = gca;
title('Depth Over Time');
xlabel('Time [s]');
ylabel('Depth [m]');
set(ah,'XLim',[1,time(end)]);
%set(ah,'YLim',[0,3]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
grid on


figure(5) % Button
area(time,Button_1,'FaceColor','b');
ah = gca;
title('Button Presses Over Time');
xlabel('Time [s]');
ylabel('Button Activation [logical]');
set(ah,'XLim',[1,time(end)]);
set(ah,'YLim',[0,1.1]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
grid on
%}

figure(6) % Encoder Stuff
plot(time,Length,'b','LineWidth',1);
ah = gca;
title('Encoder Length');
xlabel('Time [s]');
ylabel('Tether Length [m]');
set(ah,'XLim',[1,time(end)]);
%set(ah,'YLim',[0,3]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
grid on

figure(7) % Number of Satellites
plot(time,nsats,'b','LineWidth',1);
ah = gca;
title('Number of Satellites Over Time');
xlabel('Time [s]');
ylabel('Number of Satellites');
set(ah,'XLim',[1,time(end)]);
%set(ah,'YLim',[0,3]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
grid on


figure(8) % Heading [rad]
plot(time,heading,'b','LineWidth',1);
ah = gca;
title('Heading Over Time');
xlabel('Time [s]');
ylabel('Robot Heading [rad CCW from E]');
set(ah,'XLim',[1,time(end)]);
%set(ah,'YLim',[0,2*pi]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
grid on

figure(9) % Heading [Degrees]
plot(time,headingIMU,'b','LineWidth',1);
ah = gca;
title('IMU Heading Over Time');
xlabel('Time [s]');
ylabel('Robot Heading [Degrees CW from N]');
set(ah,'XLim',[1,time(end)]);
%set(ah,'YLim',[-180,180]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
grid on

figure(10) % Acceleration
plot(time,accelX,'b','LineWidth',1);
hold on
plot(time,accelY,'r','LineWidth',1);
hold on
plot(time,accelZ,'g','LineWidth',1);
%hold off
ah = gca;
title('IMU Acceleration Over Time');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
set(ah,'XLim',[1,time(end)]);
%set(ah,'YLim',[-180,180]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
legend('X Acceleration','Y Acceleration','Z Acceleration')
grid on



%{
figure(11) % Position (Pitzer Pool)
image('CData',imread('PitzerPool.png'),'XData',[0,24.68],'YData',[37.03,0])
hold on
ch11 = scatter(x,y,20,'b','filled');
ah = gca;
title('Position Over Time');
xlabel('Position East of Origin [m]');
ylabel('Position North of Origin [m]');
set(ah,'XLim',[-150,50]);
set(ah,'YLim',[0,100]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
daspect([1,1,1]);
grid on
%}

%{
figure(11) % Position (Linde Field)
image('CData',imread('LindeField.png'),'XData',[0,105.61],'YData',[137.33,0])
hold on
ch11 = scatter(x,y,20,'b','filled');
hold on
scatter([40,60,40,60,40],[80,80,60,60,80],30,'y','filled');
hold on
plot([40,60,40,60,40],[80,80,60,60,80],'y','LineWidth',1);
ah = gca;
title('Position Over Time');
xlabel('Position East of Origin [m]');
ylabel('Position North of Origin [m]');
set(ah,'XLim',[0,80]);
set(ah,'YLim',[40,100]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
daspect([1,1,1]);
grid on
%}

startTime = 5400;
endTime = 5800;
distance = sqrt((x(endTime)-x(startTime))^2+(y(endTime)-y(startTime))^2);
time = (endTime-startTime)/10;
speed = distance/time



figure(11) % Position (Dana Point)
% First water test
%Water_wpx = [60,55.5,65.5,70,80,75.5,85.5,90];
%Water_wpy = [60,45,42,57,54,39,36,51];
% Second water test
%Water_wpx = [30,25.5,35.5,40,50,45.5,55.5,60];
%Water_wpy = [80,65  ,62  ,77,74,59  ,56  ,71];
% Third water test
Water_wpx = [25,20.5,30.5,35,45,40.5,50.5,55];
Water_wpy = [80,65  ,62  ,77,74,59  ,56  ,71];
% First land test
%Land_wpx = [38,29,39,48,58,49,59,68,78,69,79,88];
%Land_wpy = [115,85,82,112,109,79,76,106,103,73,70,100];
% Second Land Test
%Land_wpx = [40,37,47,50,60,57];
%Land_wpy = [100,90,87,97,94,84];
image('CData',imread('DanaPoint.png'),'XData',[0,197.1342],'YData',[132.1058,0])
hold on
scatter(x,y,20,'b','filled');
hold on
plot([x(startTime),x(endTime)],[y(startTime),y(endTime)],'wo');
hold on
scatter(Water_wpx,Water_wpy,30,'g','filled');
hold on
plot(Water_wpx,Water_wpy,'y','LineWidth',1.5);
hold on
%plot([20,95,95,20,20],[65,65,120,120,65],'r','LineWidth',1);
ah = gca;
title('Position Over Time');
xlabel('Position East of Origin [m]');
ylabel('Position North of Origin [m]');
set(ah,'XLim',[0,197.1342]);
set(ah,'YLim',[0,132.1058]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
daspect([1,1,1]);
grid on


%{

fig = figure(12); % Depth Plot
Ih = image('CData',imread('DanaPoint.png'),'XData',[0,197.1342],'YData',[132.1058,0]);
set(Ih,'AlphaData',.7);
hold on
% Make a fit to the surface

%[xq,yq] = meshgrid(18:1:56, 55:1:80);
%[xq,yq] = meshgrid(10:.5:60, 50:.5:90);
%%%%%%%%%% Land 1
%[xq,yq] = meshgrid(20:1:90, 60:1:120);
%%%%%%%%%% Water 2
[xq,yq] = meshgrid(20:1:60, 50:1:90);
vq = griddata(x,y,-1*Depth,xq,yq);
mh = mesh(xq,yq,vq);
set(mh,'EdgeColor','k');
set(mh,'FaceColor','interp');
set(mh,'FaceAlpha',.6);

hold on
scatter3(x,y,-1*Depth,20,'b','filled');
hold on
scatter(Water_wpx,Water_wpy,30,'y','filled');
hold on
plot(Water_wpx,Water_wpy,'y','LineWidth',1.5);
hold on
for i=1:length(Water_wpx)
    plot3([Water_wpx(i),Water_wpx(i)],[Water_wpy(i),Water_wpy(i)],[0,-3],...
        'Color',[0,.5,.2],'LineWidth',3);
    hold on
end
ah = gca;
set(fig,'Position',[400,400,800,500]);
ah.Position = [.1 .2 .7 .7];
set(ah,'Color','none');
set(fig,'Color','w');
title('Depth Map');
xlabel('X Pos [m]');
ylabel('Y Pos [m]');
zlabel('Depth [m]');
%%%%%%%%%% Full Screen
%set(ah,'XLim',[0,197.1342]);
%set(ah,'YLim',[0,132.1058]);
%%%%%%%%%% Water 3
%set(ah,'XLim',[10,60]);
%set(ah,'YLim',[50,90]);
%%%%%%%%%% Water 2
set(ah,'XLim',[20,60]);
set(ah,'YLim',[50,90]);
set(ah,'ZLim',[-2.6,.5]);
set(ah,'FontSize',16);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
daspect([1,1,.15])
grid on
c = colorbar;
c.Position = [.9 .2 .03 .6];
title(c,'Depth [m]');
set(fig,'Position',[400,400,800,500]);
ah.Position = [.1 .2 .7 .7];

set(ah,'CameraViewAngleMode','Manual')
OptionZ.FrameRate=20;OptionZ.Duration=10;OptionZ.Periodic=true;
CaptureFigVid([-80,15;0,15;70,15;0,15;-80,20], 'DP3_Depth',OptionZ)
view(30,20)





fig = figure(13); % Temperature Plot Plot
set(fig,'Position',[200,200,800,500]);
Ih = image('CData',imread('DanaPoint.png'),'XData',[0,197.1342],'YData',[132.1058,14]);
set(Ih,'AlphaData',.7);
hold on
% Make a fit to the surface

%[xq,yq] = meshgrid(18:1:56, 55:1:80);
%[xq,yq] = meshgrid(10:.5:60, 50:.5:90);
%%%%%%%%%% Land 1
%[xq,yq] = meshgrid(20:1:90, 60:1:120);
%%%%%%%%%% Water 2
[xq,yq] = meshgrid(20:1:50, 60:1:86);
vq = griddata(x,y,Temperature_1,xq,yq);
mh = mesh(xq,yq,vq);
set(mh,'EdgeColor','k');
set(mh,'FaceColor','interp');
set(mh,'FaceAlpha',.6);

hold on
scatter3(x,y,Temperature_1,20,'b','filled');
hold on
scatter(Water_wpx,Water_wpy,30,'y','filled');
hold on
plot(Water_wpx,Water_wpy,'y','LineWidth',1.5);
hold on

ah = gca;
set(ah,'Color','none');
%set(fig,'Color',1.2*[70/256,150/256,180/256]);
set(fig,'Color','w');
title('Temperature Map');
xlabel('X Pos [m]');
ylabel('Y Pos [m]');
zlabel('Temperature [C]');
%%%%%%%%%% Full screen
%set(ah,'XLim',[0,197.1342]);
%set(ah,'YLim',[0,132.1058]);
%%%%%%%%%% Land 1
%set(ah,'XLim',[20,90]);
%set(ah,'YLim',[60,120]);
%%%%%%%%%% Water 3
%set(ah,'XLim',[10,60]);
%set(ah,'YLim',[50,100]);
%%%%%%%%%% Water 2
set(ah,'XLim',[20,60]);
set(ah,'YLim',[50,90]);
set(ah,'ZLim',[13.8,14.2])
set(ah,'FontSize',16)
set(ah,'TitleFontSizeMultiplier',1.2)
set(ah,'LineWidth',1)
daspect([1,1,.03])
grid on
c = colorbar;
c.Position = [.9 .2 .03 .6];
title(c,'Temp [C]');
set(fig,'Position',[400,400,800,500]);
ah.Position = [.1 .2 .7 .7];

set(ah,'CameraViewAngleMode','Manual')
%OptionZ.FrameRate=30;OptionZ.Duration=10;OptionZ.Periodic=true;
%CaptureFigVid([-80,15;0,15;70,15;0,15;-80,20], 'DP2_Temp',OptionZ)
view(30,20)





figure(14) % Tether angle
plot(time,Theta*180/pi,'b','LineWidth',1);
ah = gca;
title('Tether Angle Over Time');
xlabel('Time [s]');
ylabel('Tether Angle [Degrees]');
set(ah,'XLim',[1,time(end)]);
set(ah,'YLim',[0,90]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
grid on




figure(15) % Error of surface vehicle position
plot(time,Position_Error,'b','LineWidth',1);
ah = gca;
title('Surface Vehicle Position Error Over Time');
xlabel('Time [s]');
ylabel('Uncertaity Radius [m]');
set(ah,'XLim',[1,time(end)]);
set(ah,'YLim',[0,3]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
grid on

%}
