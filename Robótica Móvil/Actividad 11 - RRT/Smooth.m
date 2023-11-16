clear all
close all
clc

load path

%%
spath = Suavisar_Trayectoria(path);

figure
hold on
grid on

plot(path(1,:),path(2,:),'go-','LineWidth',2,'MarkerSize',10)
plot(spath(1,:),spath(2,:),'bo-','LineWidth',2,'MarkerSize',10)

title('Trayectoria suavizada')
legend('original','suavizada')

save spath
%%
ipath = Interpolacion_Simple(spath,2);

figure
hold on
grid on
plot(spath(1,:),spath(2,:),'bo-','LineWidth',2,'MarkerSize',10)
plot(ipath(1,:),ipath(2,:),'rx--','LineWidth',1.5,'MarkerSize',8)

title('Interpolacion Simple')
legend('original','interpolacion')

%%
function [pi] = Interpolacion_Simple (p,N)
    pi = [];
    
    M = size(p,2);

    for i=2:M
        X0 = p(:,i-1);
        X1 = p(:,i);

        x = linspace(X0(1),X1(1),N+2);
        y = linspace(X0(2),X1(2),N+2);

        pi = [pi [x(1:end-1); y(1:end-1)]];
    end

    pi = [pi p(:,end)];
end

function [ps] = Suavisar_Trayectoria (p)
    ps = p;
    N = 10;
    M = size(p,2);

    alpha = 0.2;
    beta = 0.6;

    for j=1:N
        for i=2:(M-1)
            ps(:,i) = ps(:,i) + alpha*(p(:,i)-ps(:,i)) + beta*(ps(:,i+1)+ps(:,i-1)-2*ps(:,i));
        end
    end
end