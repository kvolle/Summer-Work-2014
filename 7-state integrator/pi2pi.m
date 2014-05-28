function alpha = pi2pi(alpha)

alpha = rem(alpha+2*pi,2*pi);

alpha = (alpha > pi).*(alpha - 2*pi) + (alpha <= pi).*(alpha); 
