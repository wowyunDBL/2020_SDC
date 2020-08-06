%remember to do tbot = turtlebot(IP); before usage

%{
%%%execute this in the command window when things go bad

puber = rospublisher('/cmd_vel');
vmsg = rosmessage(puber);
vmsg.Angular.Z = 0;
vmsg.Linear.X = 0;
send(puber,vmsg);

%}
%{
y矩陣1 x runtime記錄了距離隨時間的變化
w矩陣1 x runtime記錄了角速度隨時間的變化
err矩陣1 x runtime記錄了距離差值隨時間的變化
%}

    %%%設定起始速度
    pub = rospublisher('/cmd_vel');
    pause(2);
    sub = rossubscriber('/scan');
    pause(2);
    velmsg = rosmessage(pub);
    velmsg.Angular.Z = 0;
    velmsg.Linear.X = 0.08;
    send(pub,velmsg);
    
    Scan_dis = getLaserScan(tbot);
    pause(0.5);
    %%% modify desired spec
    runtime = 30 ;      %%% 總時間為runtime x samplingtime
    samplingtime = 0.3;
    pre_dist = Scan_dis.Ranges(270);     %%%車子擺放的起始位置
    stopdist = 0.1;     %%%車子跑出牆壁範圍多少距離後停止
    r = Scan_dis.Ranges(270);            %%%希望車子離牆壁多少距離
    
    %%% modify pid controller
    kp = 1;
    ki = 0;
    kd = 0;
    
    %%%開啟儲存資料的矩陣
    x = zeros(1,runtime);
    y = zeros(1,runtime);
    w = zeros(1,runtime);
    od_pos =zeros(runtime,3);
    od_ori =zeros(runtime,3);
    err = zeros(1,runtime);
    
    %%% 開始control loop
    n = 1;
    tb_reset = rospublisher('/reset'); 
    resetMsg = rosmessage(tb_reset);
    send(tb_reset, resetMsg);
    while n <= runtime
        disp(['At the ',num2str(n),'th datapoint !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1']);
        [y(1,n), toofartrue]= getdis(sub,pre_dist, stopdist);
        odom = getOdometry(tbot);
        od_pos(n,:) =double(odom.Position);
        od_ori(n,:) =double(odom.Orientation);
        pre_dist = y(1,n);
        %%%離開牆而停止(數據還不能用)
        if toofartrue ==1
            %%%save test y;
            %%%break;
        end
        disp(['Distance(output of control loop)( y) = ',num2str(y(1,n))]);
        %%% proportion項
        u = r;
        err(1,n) = (-1)*(r-y(1,n));
        uc_p = kp * u;
        disp(['部份項為 ',num2str(uc_p)]);
        %%% integral項
        uc_i =0;
        if n > 2
            y1 = y(1,n-1);
            y2 = y(1,n);
            y3 = y(1,n-2);
            u1 = r - y1;
            u2 = r - y2;
            u3 = r - y3;
            u= u1 + u2 +u3;
            uc_i = ki * u;
        end
        disp(['積分項為 ',num2str(uc_i)]);
        %%% derivative項
        uc_d = 0;
        if n > 1
            y1 = y(1,n-1);
            y2 = y(1,n);
            u1 = r - y1;
            u2 = r - y2;
            uc_d = kd * ((u2 - u1)/samplingtime);
        end
        disp(['微分項為 ',num2str(uc_d)]);
        disp(['y =', num2str(y(1,n))]);
        
        %%%經過控制器，傳到plant
        uc = uc_p + uc_i + uc_d;
        w(1,n) = translr(pub,uc,samplingtime);%%%改變角速度的function
        x(1,n) = n;
        n = n+1;
        %%%無法存資料了而停止
        if n > runtime
            %%save testx x 
            save testy2 y;
            save testw2 w;
            save testerr2 err;
            figure(1)
            plot(x,y,x,w,x,err);
            legend('dstance','angular speed','err dist');
            break;
        end
    end
function w = translr(pub,uc,samplingtime)
    vel = rosmessage(pub);
    
%     vel.Angular.Z =(-1)*(uc);%%%當牆壁在+x方向的左邊時，angular velocity要乘-1
%     vel.Linear.X = 0.08;
%     disp(['Angular Velocity = ',num2str((-1)*uc)]);
%     if abs(uc) > 0.7
%         pause(1000);
%     end
%     send(pub,vel);
%     pause(samplingtime/2);
%     vel.Angular.Z =(1)*(uc);
%     vel.Linear.X = 0.08;
    vel.Angular.Z = 0.05;
    vel.Linear.X = 0.04;
    w = vel.Angular.Z;
    send(pub,vel);
end
%%% getdis可以找到車子與牆的位置
%{ 
input predist 為前一秒車與牆的距離
input stopdist為車子已經走出牆必須停下來的距離
output dist 為車子與牆的距離
output stoptrue 如果等於1，則車子已經離開牆，則車子停止
%}
function [dist, stoptrue] = getdis(sub, pre_dist, stopdist)    
    scan = receive(sub);
    cart = readCartesian(scan);
    [dist,objhead] = find_obj(pre_dist, cart , 'y');
    disp(['Object head = ',num2str(objhead)]);
    stoptrue = 0;
    if objhead < stopdist
        stoptrue = 1;
    end    
end

function [lidar_dist, head, tail, debug] = find_obj(pre_dist, lidar_range, dir)
    %=== count time
    t1 = clock;
    %=== different conditions
    if dir == 'x'
       upper = pre_dist + 0.2; lower = pre_dist - 0.2;
       mask = double((abs(lidar_range(:, 1)) < upper) & (abs(lidar_range(:, 1)) > lower));
       obj_range = [lidar_range(:, 1) .* mask lidar_range(:, 2) .* mask];
       %=== 
       obj_indice = find(obj_range(:, 1)); len = size(obj_indice); disp(len(1));
       obj_size = []; i = 1;
       while i <= len(1)
           fprintf(1, 'i = %4.0f \n', i);
           for j = i : len(1)
               if (obj_indice(j + 1) - obj_indice(j)) ~= 1
                   obj_size = [obj_size; [i j j-i+1]]; disp('obj_size = '); disp(obj_size);
                   i = j + 1; 
                   break;
               elseif j == len(1)-1
                   j = j + 1;
                   break;
               end
           end
           
           if i == len(1)
              obj_size = [obj_size; [i i 1]]; break;
           end
           
           if j == len(1)
               obj_size = [obj_size; [i j j-i+1]]; break;
           end
       end
       disp('obj_range = '); disp(obj_range); 
       disp('obj_size = '); disp(obj_size);
       disp('obj_indice = '); disp(obj_indice);
       debug = obj_size;
       [val, row] = max(obj_size(:, 3)); head = obj_size(row, 1); tail = obj_size(row, 2);
       %===
       lidar_dist = abs(mean(obj_range(obj_indice(head):obj_indice(tail), 1)));
       head = obj_range(obj_indice(head), 2); tail = obj_range(obj_indice(tail), 2);
       %=== count time
       t2 = clock; fprintf('total time count = %4.8f', etime(t2, t1));
    elseif dir == 'y'
       upper = pre_dist + 0.2; lower = pre_dist - 0.2;
       mask = double((abs(lidar_range(:, 2)) < upper) & (abs(lidar_range(:, 2)) > lower));
       obj_range = [lidar_range(:, 1) .* mask lidar_range(:, 2) .* mask];
       %=== find_obj(0.1, scan_data.readCartesian, 'x')
       obj_indice = find(obj_range(:, 2)); len = size(obj_indice); disp(len(1));
       obj_size = []; i = 1;
       while i <= len(1)
           fprintf(1, 'i = %4.0f \n', i);
           for j = i : len(1)
               if (obj_indice(j + 1) - obj_indice(j)) ~= 1
                   obj_size = [obj_size; [i j j-i+1]]; %disp('obj_size = '); disp(obj_size);
                   i = j + 1; 
                   disp('a0');
                   break;
               elseif j == len(1)-1
                   j = j + 1;
                   disp('a1');
                   break;
               end
           end
           
           if i == len(1)
              obj_size = [obj_size; [i i 1]]; disp('b0');
              break;
           end
           
           if j == len(1)
               obj_size = [obj_size; [i j j-i+1]]; disp('b1');
               break;
           end
           
       end
       disp('obj_range = '); disp(obj_range); 
       disp('obj_size = '); disp(obj_size);
       disp('obj_indice = '); disp(obj_indice);
       debug = obj_size;
       [val, row] = max(obj_size(:, 3)); head = obj_size(row, 1); tail = obj_size(row, 2);
       %===
       lidar_dist = abs(mean(obj_range(obj_indice(head):obj_indice(tail), 2)));
       head = obj_range(obj_indice(head), 1); tail = obj_range(obj_indice(tail), 1);
       %=== count time
       t2 = clock; fprintf('total time count = %4.8f', etime(t2, t1));
    else
        disp('wrong dir input');
        lidar_dist = NaN; head = NaN; tail = NaN;
        %=== count time
        t2 = clock; fprintf('total time count = %4.8f', etime(t2, t1));
        return;
    end
    
end


