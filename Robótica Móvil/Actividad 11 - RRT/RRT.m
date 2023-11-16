clear all
close all
clc

xl = [-5 -5]';
xu = [5 5]';

xo = [0.5 1.5; 1.5 1.5; 2.5 0.5]';
ro = [0.20 0.20 0.20];

x0 = [0 0]';
xd = [3 3]';

D = 0.3;
N = 1000;

M = x0;
s = [];
t = [];

j = 2;

figure
hold on
grid on

while j<N
	xr = xl + (xu-xl).*rand(2,1);
    m = size(M,2);

	dm = zeros(1,m); 
    for i=1:m
        xi = M(:,i);
        dm(i) = norm(xi-xr); 
    end

    [~,i_min] = min(dm);

    xc = M(:,i_min);
    v = (xr-xc)/norm(xr-xc);
    xn = xc + D*v;
    
    obs_free = true;
    for k=1:size(xo,2)
        d = norm(xn-xo(:,k));
        
        if d<=ro(k)
            obs_free = false;
        end
    end

    if obs_free
        M = [M xn];
        s = [s i_min];
        t = [t j];

        % solo para graficar
        line([xc(1) xn(1)],[xc(2) xn(2)],'Color','b');
%         plot(M(1,:),M(2,:),'bo','MarkerSize',10)
%         drawnow

        if(norm(xd-xn)<D)
            M = [M xd];
            
            % solo para graficar
            line([xn(1) xd(1)],[xn(2) xd(2)],'Color','b');

            s = [s j];
            t = [t j+1];
            break
        end

        j = j + 1;
    end
end

plot(M(1,:),M(2,:),'bo','MarkerSize',10)
plot(x0(1),x0(2),'rx','MarkerSize',10,'LineWidth',2)
plot(xd(1),xd(2),'gx','MarkerSize',10,'LineWidth',2)
axis equal
xlabel('x')
ylabel('y')

theta = linspace(-pi,pi,50);
for i=1:size(xo,2)
    plot(xo(1,i)+ro(i)*cos(theta),xo(2,i)+ro(i)*sin(theta),'r-','LineWidth',2)
end

% trayectoria en grafo
node = t(end);
path = M(:,node);

while node~=1
    node = s(t==node);
    path = [M(:,node) path];
end

plot(path(1,:),path(2,:),'g-','LineWidth',3)

save path.mat
