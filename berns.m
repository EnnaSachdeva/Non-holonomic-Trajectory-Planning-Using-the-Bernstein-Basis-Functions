function [val]=berns(n,t)
%%Assuming 5th order polynomial
val=0;
% mu=(1-(t/10));
mu=t/10;
% val=(factorial(5)/(factorial(n)*factorial(5-n)))*((1-mu)^(5-n))*mu^(n);
val=nchoosek(5,n)*((1-mu)^(5-n))*(mu^n);

end