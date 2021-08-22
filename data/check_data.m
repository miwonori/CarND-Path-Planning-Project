T = readtable('highway_map.csv');
%%
figure;hold on;
plot(T.Var1,T.Var2,'.-');
plot(T.Var1(1),T.Var2(2),'or');
text(T.Var1(1),T.Var2(2),'Start');

 X = 909.48; Y = 1128.67;
plot(X,Y,'dm');
text(X,Y,'Car Position');

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
  