% Plots calibration curves for all analog sensors
clear
clf
% Temperature Sensor Data
Temp = [24.2,20.9,21.3,14.2,12.7,11.1,10.7,19.1,17.7,16.6,15.1]';
T_sense = [964,728,763,280,173,62,31,613,520,437,344]';
% Battery Current Sensor Data
Current = [0.09,0.084,0.417,0.75,1.394,0.526,0.305]';
C_sense = [32,30,136,241,444,170,100]';
% Battery Voltage Sensor Data
Voltage = [12.92,11.99,11.49,10.97,10.48,10.03,9.47,8.98,8.46,7.99,7.52,6.99]';
V_sense = [1003,930,892,852,815,779,735,698,658,621,585,544]';
% Pressure Sensor Data
Depth = .01*[0,20,40,60,80,100,120,140]';
D_sense = [103,117,131,146,161,176,192,208]';
% Encoder Data
EncoderL = [0.0,0.2,0.4,0.6,0.8,1.0,1.2,1.4,1.6,1.8,2.0,2.2,2.4,2.6,2.8,...
3.0,3.2,3.4,3.6,3.8,4,4.2,4.4,4.6]';
E_sense = [0,8,16,24,32,41,50,59,68,77,86,95,105,114,124,134,144,154,...
164,175,186,197,208,219]';


[T_fit,T_gof] = fit(T_sense,Temp,'poly1');
[C_fit,C_gof] = fit(C_sense,Current,'poly1');
[V_fit,V_gof] = fit(V_sense,Voltage,'poly1');
[D_fit,D_gof] = fit(D_sense,Depth,'poly1');
[E_fit,E_gof] = fit(E_sense,EncoderL,'poly2')

T_error = 0.1;
C_error = .01;
V_error = .02;
D_error = .01;
E_error = .01;


% Plot Temperature calibration curve
figure(1)
errorbar(T_sense,Temp,T_error*ones(length(Temp),1),'k','LineStyle','none','Capsize',10);
hold on
ah = gca;
plot_x = linspace(0,1023,100);
str = strcat('y = ',num2str(T_fit.p1),'*x + ',num2str(T_fit.p2),',  R^2: ',num2str(T_gof.rsquare));
annotation('textbox',[.2 .5 .2 .2],'String',str,'FitBoxToText','on');
plot(plot_x,T_fit(plot_x),'b--','Linewidth',1);
title('Temperature Sensor Calibration Curve')
xlabel('Teensy ADC Output [0-1023 => 0-3.3V]');
ylabel('Measured Temperature [C]');
set(ah,'XLim',[0,1023]);
set(ah,'YLim',[9,26]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
legend('Collected Data','Linear Fit Line');
grid on


% Plot Current calibration curve
figure(2)
errorbar(C_sense,Current,C_error*ones(length(Current),1),'k','LineStyle','none','Capsize',10);
hold on
ah = gca;
plot_x = linspace(0,1023,100);
str = strcat('y = ',num2str(C_fit.p1),'*x + ',num2str(C_fit.p2),',  R^2: ',num2str(C_gof.rsquare));
annotation('textbox',[.2 .5 .2 .2],'String',str,'FitBoxToText','on');
plot(plot_x,C_fit(plot_x),'b--','Linewidth',1);
title('Current Sensor Calibration Curve')
xlabel('Teensy ADC Output [0-1023 => 0-3.3V]');
ylabel('Measured Current [A]');
set(ah,'XLim',[0,1023]);
set(ah,'YLim',[0,3]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
legend('Collected Data','Linear Fit Line');
grid on


% Plot Current calibration curve
figure(3)
errorbar(V_sense,Voltage,V_error*ones(length(Voltage),1),'k','LineStyle','none','Capsize',10);
hold on
ah = gca;
plot_x = linspace(0,1023,100);
str = strcat('y = ',num2str(V_fit.p1),'*x + ',num2str(V_fit.p2),',  R^2: ',num2str(V_gof.rsquare));
annotation('textbox',[.2 .5 .2 .2],'String',str,'FitBoxToText','on');
plot(plot_x,V_fit(plot_x),'b--','Linewidth',1);
title('Voltage Sensor Calibration Curve')
xlabel('Teensy ADC Output [0-1023 => 0-3.3V]');
ylabel('Measured Voltage [V]');
set(ah,'XLim',[0,1023]);
set(ah,'YLim',[0,13]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
legend('Collected Data','Linear Fit Line');
grid on


% Plot Current calibration curve
figure(4)
errorbar(D_sense,Depth,D_error*ones(length(Depth),1),'k','LineStyle','none','Capsize',10);
hold on
ah = gca;
plot_x = linspace(0,300,100);
str = strcat('y = ',num2str(D_fit.p1),'*x + ',num2str(D_fit.p2),',  R^2: ',num2str(D_gof.rsquare));
annotation('textbox',[.2 .5 .2 .2],'String',str,'FitBoxToText','on');
plot(plot_x,D_fit(plot_x),'b--','Linewidth',1);
title('Pressure Sensor Calibration Curve')
xlabel('Teensy ADC Output [0-1023 => 0-3.3V]');
ylabel('Depth [m]');
set(ah,'XLim',[0,300]);
set(ah,'YLim',[0,2]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
legend('Collected Data','Linear Fit Line');
grid on


% Plot Encoder calibration curve
figure(5)
errorbar(E_sense,EncoderL,E_error*ones(length(EncoderL),1),'k','LineStyle','none','Capsize',10);
hold on
ah = gca;
plot_x = linspace(0,230,100);
str = strcat('y = ',num2str(E_fit.p1),'x^2 + ',num2str(E_fit.p2),'x + ',...
num2str(E_fit.p3),',  R^2: ',num2str(E_gof.rsquare));
annotation('textbox',[.2 .5 .3 .3],'String',str,'FitBoxToText','on');
plot(plot_x,E_fit(plot_x),'b--','Linewidth',1);
title('Encoder Calibration Curve')
xlabel('Discrete Encoder Output');
ylabel('Depth [m]');
set(ah,'XLim',[0,230]);
set(ah,'YLim',[0,5]);
set(ah,'FontSize',12);
set(ah,'TitleFontSizeMultiplier',1.2);
set(ah,'LineWidth',1);
legend('Collected Data','Quadratic Fit Line');
grid on

figure(6)

EncoderFitL = E_fit(E_sense);
%plot(E_sense,EncoderFitL,'o')
EncoderError = EncoderL-EncoderFitL;
plot(E_sense,EncoderError,'o')




