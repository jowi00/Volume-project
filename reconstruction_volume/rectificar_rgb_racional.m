function rgbRect = rectificar_rgb_racional(rgb, fx, fy, cx, cy, D)

% Rectifica una imagen RGB con el modelo racional de ROS/OpenCV.
% La salida conserva la misma resolucion y la misma matriz intrinseca.

if numel(D) ~= 8
    error('D debe contener 8 coeficientes del modelo rational_polynomial')
end

[h,w,channels] = size(rgb);
[u,v] = meshgrid(1:w,1:h);

x = (u-1-cx) / fx;
y = (v-1-cy) / fy;

r2 = x.^2 + y.^2;
r4 = r2.^2;
r6 = r2.^3;

k1 = D(1);
k2 = D(2);
p1 = D(3);
p2 = D(4);
k3 = D(5);
k4 = D(6);
k5 = D(7);
k6 = D(8);

radial = (1+k1*r2+k2*r4+k3*r6) ./ (1+k4*r2+k5*r4+k6*r6);

xDist = x.*radial + 2*p1*x.*y + p2*(r2+2*x.^2);
yDist = y.*radial + p1*(r2+2*y.^2) + 2*p2*x.*y;

uDist = fx*xDist + cx + 1;
vDist = fy*yDist + cy + 1;

rgbRect = zeros(size(rgb),'like',rgb);

for c = 1:channels
    channel = interp2(double(rgb(:,:,c)),uDist,vDist,'linear',0);
    rgbRect(:,:,c) = cast(channel,'like',rgb);
end

end
