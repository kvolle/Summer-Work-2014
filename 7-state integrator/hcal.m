function Y = hcal(X);

global gyrog

[m,n] = size(X);

for i = 1 : n
    
    phi = X(1,i);
    th  = X(2,i);
    psi = X(3,i);
    p   = gyrog(1);
    q   = gyrog(2);
    r   = gyrog(3);
    Z   = X(4,i);
    u   = X(5,i);
    v   = X(6,i);
    w   = X(7,i);
    h = [ r*v - q*w - (mu*u)/m
        p*w - r*u - (mu*v)/m
        q*u - p*v - (FT(1))/m
        u
        v
        -Z];

    Y(:,i) = h;
    
end