rng(0)

rician = true;
ric_k = 3;
delta_t = 1;

M  = 15;
N = 90;
v_max = 50;
S_min = 25;

ch_power_gain = -50;
alpha = 2.7;
error = 0.001;
noise_power = -110;
H = 100;
P = 0.1;

data_generation = [54 49 21 47 52 66 1 36 22 24 63 1 61 35 1];
data_generation = [1 1 1 1 1 1 1 1 1 1 1 1 1 1 1];
deadline = [74 69 41 67 72 86 16 56 42 44 83 2 81 55 21];

device_X = [0 50 120 210 170 310 170 175 270 320 480 650 725 750 760];
device_Y =[180 350 550 310 0 0 680 685 690 750 560 410 380 370 600];


C=[mean(device_X) mean(device_Y)];
% l_dist = 0
% for i=1:M
%     temp = sqrt((device_X(i)-C(1))^2 + (device_Y(i)-C(2))^2);
%     if temp >= l_dist
%         l_dist = temp
%     end
% end

r=400; % radius
l_dist = r;
theta=0:2*pi/360:2*pi; % the angle
m=[l_dist*cos(theta')+C(1) l_dist*sin(theta')+C(2)]; % the points you asked


X_r = [];
Y_r = [];
B_r = [];
G_r =[];
for n=1:N
   X_r(n) = m(n*4,1);
   Y_r(n) = m(n*4,2);
end
% Y_
for i=1:M
    for n=1:N
        B_r(i,n) = 0;
        G_r(i,n) = 0;
    end
end

h = [];
for i=1:M
    for n=1:N
%         temp = randi([0,1])
%         if temp == 0
%             temp = -1
%         end
        temp = 1;
        h(i,n) = sqrt((ric_k/(ric_k+1))* randi([-1,1])) + sqrt((ric_k/(ric_k+1)) * ((1/sqrt(2))*(randn + 1i*randn)));
    end
end

if rician == false
    
    for i=1:M
        for n=1:N
            h(i,n) = 1;
        end
    end
end

prev_obj = 100; % dummy number for making sure the first iteration's loop is running
cvx_optval = 50; % dummy number for making sure the first iteration's loop is running
r = 1;

while (prev_obj -  cvx_optval) >= error || isnan(prev_obj -  cvx_optval)
    disp(strcat('Starting iteration : ',num2str(r),' ....'))
    prev_obj = cvx_optval;
    cvx_clear
    cvx_begin
       variable K(M,1);
       variable C(M, N);
       variable B(M, N)
       variable G(M,N)
       variable X(N)
       variable Y(N)

       % Objective
       obj = 0;
       for i=1:M
        obj = obj + K(i);
       end
       maximize(obj)

       % Contraints
       subject to

        % 15b
        for i=1:M
          c_sum = 0;
          for n=data_generation(i):deadline(i)
            c_sum = c_sum + C(i,n);
          end
          delta_t*c_sum >= K(i)*S_min;
        end

        % 16 ???
        for i=1:M
            for n=data_generation(i):deadline(i)
                -(((B_r(i, n) + G_r(i,n))^2)/4) ...
                - (((B_r(i, n) - G_r(i,n))*(B(i, n) - B_r(i, n) + G(i,n) - G_r(i,n)))/2) ...
                + (((B(i, n) - G(i,n))^2)/4) ...
                + C(i, n) <= 0
            end
        end

    %     15d
        for i=1:M
            for n=data_generation(i):deadline(i)
                  G(i,n) <= -(alpha*((P*ch_power_gain*(abs(h(i,n))^2))/noise_power)*log2(exp(1))) / ...
                            (2*((H^2 + (device_X(i) - X_r(n))^2 + (device_Y(i) - Y_r(n))^2)^(alpha/2) + ((P*ch_power_gain*(abs(h(i,n))^2))/noise_power))) ...
                            *(1/((H^2 + (device_X(i) - X_r(n))^2 + (device_Y(i) - Y_r(n))^2))) ... % end A ...
                            * ( (device_X(i) - X(n))^2 + (device_Y(i) - Y(n))^2 ...
                            - (device_X(i) - X_r(n))^2 - (device_Y(i) - Y_r(n))^2)...
                            + log2(1 + ((P*ch_power_gain*(abs(h(i,n))^2))/(noise_power*((H^2 + (device_X(i) - X_r(n))^2 + (device_Y(i) - Y_r(n))^2))^alpha/2)))  % B
            end
        end

        % 15e
        for i=1:M
            0<=K(i)<=1;
        end

        % 15f
        for n=1:N
            for i=1:M
                0<= B(i,n) <= K(i);
            end
        end

        % 2
        for n=1:N-1
            ((X(n+1) - X(n))^2) + ((Y(n+1) - Y(n))^2) <= (v_max*delta_t)^2
        end

        % 6
        
        for n=1:N
            b_sum = 0;
            for i=1:M
                b_sum = b_sum + B(i, n);
            end
            b_sum <= 1;
        end

        % 10f
        X(1) == 0
        Y(1) == 400

        % 10g
        X(N) == 800
        Y(N) == 400

    cvx_end
%     break
%     if (prev_obj - cvx_optval) <error
%         break
%     end
    
    for n=1:N
       X_r(n) = X(n);
       Y_r(n) = Y(n);
    end
    for i=1:M
        for n=1:N
            B_r(i,n)= B(i,n);
            G_r(i,n) = G(i,n);
        end
    end
    
    r = r+1;
       
end % while


hold on; scatter(X, Y, 10, 'b');scatter(device_X, device_Y,50, 'r'); xlim([0 800]);ylim([0 800]); text(device_X, device_Y, split(num2str(deadline))); hold off
