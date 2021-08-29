T = readtable('highway_map.csv');
index = 0:1:length(T.Var1)-1;
%% Cartensian Coordinate
figure;hold on;
plot(T.Var1,T.Var2,'.-');
text(T.Var1,T.Var2,num2str(index'));
plot(T.Var1(1),T.Var2(2),'or');
text(T.Var1(1),T.Var2(2),'Start');

start_X = 909.48; start_Y = 1128.67;
plot(start_X,start_Y,'dm');
text(start_X,start_Y,'Car Position');

%%  Frenet Coordinate
figure;hold on;
% plot(T.Var4,T.Var3,'.-');
plot(T.Var4+6,T.Var3,'.-');
text(T.Var4+6,T.Var3,num2str(index'));

start_S = 124.834; start_D = 6.16483;
plot(start_D,start_S,'dm');
text(start_D,start_S,'Car Position');

%%
 X = 915.881;  Y = 1126.89;
 plot(X,Y,'.r');
 X = 922.266;  Y = 1126.95;
  plot(X,Y,'.r');
 X = 928.651;  Y = 1127;
  plot(X,Y,'.r');
 X = 935.037;  Y = 1127.06;
  plot(X,Y,'.r');

%%
X = 909.896; Y = 1128.52;
plot(X,Y,'ok');
  X = 910.296,  Y = 1128.38;
  plot(X,Y,'ok');
  X = 910.696;  Y = 1128.24;
  plot(X,Y,'ok');
  X = 911.096;  Y = 1128.1;
  plot(X,Y,'ok');
  X = 911.496;  Y = 1127.97;
  plot(X,Y,'ok');
  X = 911.896;  Y = 1127.84;
  plot(X,Y,'ok');
  X = 912.296;  Y = 1127.71;
  plot(X,Y,'ok');
  X = 912.696;  Y = 1127.59;
  plot(X,Y,'ok');
  X = 913.096;  Y = 1127.48;
  plot(X,Y,'ok');
  X = 913.496;  Y = 1127.37;
  plot(X,Y,'ok');
  
  %%
X = 1080.96; Y = 1174.44; plot(X,Y,'ok');
X = 1081.35; Y = 1174.58; plot(X,Y,'om');
X = 1080.86; Y = 1174.41; plot(X,Y,'ok');
X = 1081.25; Y = 1174.55; plot(X,Y,'om');
X = 1103.5; Y = 1180.03; plot(X,Y,'ok');
X = 1127.92; Y = 1183.17; plot(X,Y,'om');
  