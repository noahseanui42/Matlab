function xyz = scan_grid(xr, yr, zr, nx, ny, nz)
%#codegen
xs = linspace(xr(1), xr(2), nx);
ys = linspace(yr(1), yr(2), ny);
zs = linspace(zr(1), zr(2), nz);

xyz = zeros(nx*ny*nz, 3);
k = 0;
for iz = 1:nz
    for jy = 1:ny
        for ix = 1:nx
            if mod(jy,2) == 1
                xi = ix;                % forward
            else
                xi = nx - ix + 1;       % reverse
            end
            k = k + 1;
            xyz(k,:) = [xs(xi), ys(jy), zs(iz)];
        end
    end
end