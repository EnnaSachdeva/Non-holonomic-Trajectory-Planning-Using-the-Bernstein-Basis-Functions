function [val]=berns_dot(n,t)
%%Assuming 5th order polynomial
val=0;
% mu=(1-(t/10));
mu=t/10;
def=0;
% nchoosek(5,n)
temp=(1/10)*nchoosek(5,n);
if n==0
    def=5*((1-mu)^4)*(-1);
    def=def*nchoosek(5,0);
elseif n==1
    def=4*((1-mu)^3)*(-1)*(mu)+(1-mu)^4;
    def=def*nchoosek(5,1);
elseif n==2
    def=3*((1-mu)^2)*(-1)*((mu)^2)+2*((1-mu)^3)*mu;
    def=def*nchoosek(5,2);
elseif n==3
    def=2*(1-mu)*(-1)*(mu^3) + 3*((1-mu)^2)*(mu^2);
    def=def*nchoosek(5,3);
elseif n==4
    def=(-1)*(mu^4)+4*(mu^3)*(1-mu);
    def=def*nchoosek(5,4);
else 
    def=5*mu^4;
    def=def*nchoosek(5,5);
end
   val=def;
end