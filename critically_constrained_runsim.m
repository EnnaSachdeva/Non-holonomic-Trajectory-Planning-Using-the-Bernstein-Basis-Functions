%%%%%NONHOLONOMIC MOTION PLANNING USING BERNSTEIN POLYNOMIAL%%%%%%%%%%
close all
clear all

%%%%% Given parameters
x0=10;
xf=50;

y0=10;
yf=50;

t0=0;
tf=10;
tc=5;
%%%%% Inferred parameters : W_x0=x0, W_x5=xf, W_k0=k0, W_k5=kf
W_x0=10;
W_x5=50;

W_k0=0;  
W_k5=0;

%%DEFINE THE A_x MATRIX (for solving A_x=B_x*W_x[])
A_x=[0 0 5 -25 23.125 -1.25]';
B_x=[0 0 0 0 ;...    
   0 0 0 0 ;...
   (1/2) 0 0 0 ;...
   0 0 0 (-1/2);
   5/32 10/32 10/32 5/32;... 
   berns_dot(1,tc) berns_dot(2,tc) berns_dot(3,tc) berns_dot(4,tc)];
W_x=[0 0 0 0];
W_x=pinv(B_x(3:6,:))*A_x(3:6);
% W_x=A_x(3:6)'/B_x(3:6,:)';
% W_x=W_x';
%%%getting the coefficients vectors F0 and Ff
F0=zeros(1,6);
Ff=zeros(1,6);
Fc=zeros(1,6);
[ F0(1) F0(2) F0(3) F0(4) F0(5) F0(6)]=get_coeff(W_x0,W_x(1),W_x(2),W_x(3),W_x(4),W_x5,t0,t0,tf);

[ Ff(1) Ff(2) Ff(3) Ff(4) Ff(5) Ff(6)]=get_coeff(W_x0,W_x(1),W_x(2),W_x(3),W_x(4),W_x5,t0,tf,tf);
[ Fc(1) Fc(2) Fc(3) Fc(4) Fc(5) Fc(6)]=get_coeff(W_x0,W_x(1),W_x(2),W_x(3),W_x(4),W_x5,t0,tc,tf);
%DEFINE the A_k matrix for solving A_k=B_k*W_k including constraints for Y
Yt=[];
T=[];
X=[];
Xdot=[];
Theta=[];
Ydot=[];

    A_k=[1 1 15 40]';
B_k=[ (1/2) 0 0 0;...
         0 0 0 (-1/2) ;...
%     F0(2) F0(3) F0(4) F0(5);...         
         Fc(2) Fc(3) Fc(4) Fc(5);... 
         Ff(2) Ff(3) Ff(4) Ff(5)];%%where F0=f(t0) and Ff=f(tf)

     W_k=[0 0 0 0]';
     W_k=pinv(B_k)*A_k
    Ft=zeros(1,6); 
for i=0:0.1:10
   
    
    [ Ft(1) Ft(2) Ft(3) Ft(4) Ft(5) Ft(6)]=get_coeff(W_x0,W_x(1),W_x(2),W_x(3),W_x(4),W_x5,t0,i,tf);
    Ft = [Ft(1) Ft(2) Ft(3) Ft(4) Ft(5) Ft(6)];
  
%  yt=y0 + Ft*[0 W_k(1) W_k(2) W_k(3) W_k(4) 0]';
   yt=y0 + Ft*[W_k0 W_k(1) W_k(2) W_k(3) W_k(4) W_k5]';
    
   Yt=[Yt yt];
   
   T=[T i];

   x_t=[W_x0 W_x(1) W_x(2) W_x(3) W_x(4) W_x5]*[berns(0,i) berns(1,i) berns(2,i) berns(3,i) berns(4,i) berns(5,i)]';
  X = [X x_t];
    
%   b= berns_dot(0,i) +berns_dot(1,i) +berns_dot(2,i) +berns_dot(3,i)+ berns_dot(4,i)+ berns_dot(5,i);
  [berns_dot(0,i) berns_dot(1,i) berns_dot(2,i) berns_dot(3,i) berns_dot(4,i) berns_dot(5,i)]
  xdot_t=[W_x0 W_x(1) W_x(2) W_x(3) W_x(4) W_x5]*[berns_dot(0,i) berns_dot(1,i) berns_dot(2,i) berns_dot(3,i) berns_dot(4,i) berns_dot(5,i)]'
  Xdot=[Xdot xdot_t];
  
  tan_t=[0 W_k(1) W_k(2) W_k(3) W_k(4) 0]*[berns(0,i) berns(1,i) berns(2,i) berns(3,i) berns(4,i) berns(5,i)]';
  Theta =[Theta atan(tan_t)];
  %%ydot_t=xdot_t*tan(Theta)
   ydot_t=xdot_t*tan_t;
   Ydot=[Ydot ydot_t];
   
end
figure(1)
scatter( T,Yt,'filled')
title('Change in Y coordinate WRT time(Critically constrained)');
xlabel('time');
ylabel('Y-coordinate');
hold on
% % 
% figure(2)
% % % 
figure(2)
scatter(T,X,'filled')
title('Change in x coordinate WRT time(Critically constrained)');
xlabel('time');
ylabel('X-coordinate');
hold on
% % 
figure(3)
scatter(T,Xdot, 'filled')
title('Change in x_dot coordinate WRT time(Critically constrained)');
xlabel('time');
ylabel('Xdot-coordinate');

hold on
% % 
figure(4)
scatter(T,Ydot, 'filled')
title('Change in Ydot coordinate WRT time(Critically constrained)');
xlabel('time');
ylabel('Ydot-coordinate');
hold on
% % 
figure(5)
% 
scatter(T,Theta, 'filled')
title('Change in Theta coordinate WRT time(Critically constrained)');
xlabel('time');
ylabel('Theta-coordinate');

hold on
% % 
figure(6)
scatter(Yt,X,'filled')
title('Change in X coordinate WRT Y coordinate(Critically constrained)');
xlabel('Y-coordinate');
ylabel('X-coordinate');
hold on
