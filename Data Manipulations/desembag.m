function desembag(filtro)
    bag = rosbag('~/Documents/Git/UR3-Modelation/MAT files/2019-10-18-16-14-06.bag');
    msg = readMessages(bag);

    time = zeros(length(msg),1);
    Pos1 = zeros(length(msg),1);
    Vel1 = zeros(length(msg),1);
    T1 = zeros(length(msg),1);
    Pos2 = zeros(length(msg),1);
    Vel2 = zeros(length(msg),1);
    T2 = zeros(length(msg),1);
    Pos3 = zeros(length(msg),1);
    Vel3 = zeros(length(msg),1);
    T3 = zeros(length(msg),1);
    Pos4 = zeros(length(msg),1);
    Vel4 = zeros(length(msg),1);
    T4 = zeros(length(msg),1);
    Pos5 = zeros(length(msg),1);
    Vel5 = zeros(length(msg),1);
    T5 = zeros(length(msg),1);
    Pos6 = zeros(length(msg),1);
    Vel6 = zeros(length(msg),1);
    T6 = zeros(length(msg),1);

    for i=1:length(msg)
        time(i) = (msg{i,1}.Header.Stamp.Nsec - msg{1,1}.Header.Stamp.Nsec)/1e9 + (msg{i,1}.Header.Stamp.Sec - msg{1,1}.Header.Stamp.Sec);
        Pos1(i) = msg{i,1}.Position(1,1);
        Vel1(i) = msg{i,1}.Velocity(1,1);
        T1(i) = msg{i, 1}.Effort(1,1);
        Pos2(i) = msg{i,1}.Position(2,1);
        Vel2(i) = msg{i,1}.Velocity(2,1);
        T2(i) = msg{i, 1}.Effort(2,1);
        Pos3(i) = msg{i,1}.Position(3,1);
        Vel3(i) = msg{i,1}.Velocity(3,1);
        T3(i) = msg{i, 1}.Effort(3,1);
        Pos4(i) = msg{i,1}.Position(4,1);
        Vel4(i) = msg{i,1}.Velocity(4,1);
        T4(i) = msg{i, 1}.Effort(4,1);
        Pos5(i) = msg{i,1}.Position(5,1);
        Vel5(i) = msg{i,1}.Velocity(5,1);
        T5(i) = msg{i, 1}.Effort(5,1);
        Pos6(i) = msg{i,1}.Position(6,1);
        Vel6(i) = msg{i,1}.Velocity(6,1);
        T6(i) = msg{i, 1}.Effort(6,1);
    end
    dados = [time,Pos1,Vel1,T1,Pos2,Vel2,T2,Pos3,Vel3,T3,Pos4,Vel4,T4,Pos5,Vel5,T5,Pos6,Vel6,T6];
    
    if filtro == 1
        [zb,pb,kb] = butter(2,2*pi*6,'s');
        [a,b] = zp2tf(zb,pb,kb);
        duracao = dados(end,1) + 0.008;

        close all

        dados_filtro(1,:) = dados(:,1);
        dados_filtro(2,:) = dados(:,4);
        dado = dados_filtro;
        save('untitled.mat','dado');
        simout = sim('ButterWorth_filter','SrcWorkspace','current');
        T1a = T1;
        T1 = simout.simout;
        figure(1)
        plot(time,T1a);
        hold on
        plot(time,T1);
        grid on

        [zb,pb,kb] = butter(2,2*pi*6,'s');
        [a,b] = zp2tf(zb,pb,kb);

        dados_filtro(1,:) = dados(:,1);
        dados_filtro(2,:) = dados(:,7);
        dado = dados_filtro;
        save('untitled.mat','dado');
        simout = sim('ButterWorth_filter','SrcWorkspace','current');
        T2a = T2;
        T2 = simout.simout;
        figure(2)
        plot(time,T2a);
        hold on
        plot(time,T2);
        grid on

        [zb,pb,kb] = butter(2,2*pi*6,'s');
        [a,b] = zp2tf(zb,pb,kb);
        dados_filtro(1,:) = dados(:,1);
        dados_filtro(2,:) = dados(:,10);
        dado = dados_filtro;
        save('untitled.mat','dado');
        simout = sim('ButterWorth_filter','SrcWorkspace','current');
        T3a = T3;
        T3 = simout.simout;
        figure(3)
        plot(time,T3a);
        hold on
        plot(time,T3);
        grid on

        [zb,pb,kb] = butter(2,2*pi*6,'s');
        [a,b] = zp2tf(zb,pb,kb);
        dados_filtro(1,:) = dados(:,1);
        dados_filtro(2,:) = dados(:,13);
        dado = dados_filtro;
        save('untitled.mat','dado');
        simout = sim('ButterWorth_filter','SrcWorkspace','current');
        T4a = T4;
        T4 = simout.simout;
        figure(4)
        plot(time,T4a);
        hold on
        plot(time,T4);
        grid on

        [zb,pb,kb] = butter(2,2*pi*6,'s');
        [a,b] = zp2tf(zb,pb,kb);
        dados_filtro(1,:) = dados(:,1);
        dados_filtro(2,:) = dados(:,16);
        dado = dados_filtro;
        save('untitled.mat','dado');
        simout = sim('ButterWorth_filter','SrcWorkspace','current');
        T5a = T5;
        T5 = simout.simout;
        figure(5)
        plot(time,T5a);
        hold on
        plot(time,T5);
        grid on


        [zb,pb,kb] = butter(2,2*pi*10,'s');
        [a,b] = zp2tf(zb,pb,kb);
        dados_filtro(1,:) = dados(:,1);
        dados_filtro(2,:) = dados(:,19);
        dado = dados_filtro;
        save('untitled.mat','dado');
        simout = sim('ButterWorth_filter','SrcWorkspace','current');
        T6a = T6;
        T6 = simout.simout;
        figure(6)
        plot(time,T6a);
        hold on
        plot(time,T6);
        grid on

        delete('untitled.mat');
        dados = [time,Pos1,Vel1,T1,Pos2,Vel2,T2,Pos3,Vel3,T3,Pos4,Vel4,T4,Pos5,Vel5,T5,Pos6,Vel6,T6];
    end
    save ('/home/pedro/Documents/Git/UR3-Modelation/MAT files/Id_data','dados');
end