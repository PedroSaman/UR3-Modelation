function desembag(filepath,outputpath)
    bag = rosbag(filepath);
    %exemple: '~/Documents/Git/UR3-Modelation/MAT files/Bag_Files/Apenas_junta_6.bag'
    msg = readMessages(bag);
    filtro = 0;
    
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
    data = [time,Pos1,Vel1,T1,Pos2,Vel2,T2,Pos3,Vel3,T3,Pos4,Vel4,T4,Pos5,Vel5,T5,Pos6,Vel6,T6];
    
     if filtro == 1
         [zb,pb,kb] = butter(2,2*pi*6,'s');
         [a,b] = zp2tf(zb,pb,kb);
         duracao = data(end,1) + 0.008;
 
         close all
 
         dados_filtro(1,:) = data(:,1);
         dados_filtro(2,:) = data(:,4);
         dado = dados_filtro;
         save('untitled.mat','dado');
         simout = sim('ButterWorth_filter','SrcWorkspace','current');
         T1 = simout.simout;
         T1 = T1(1:end-1);
 
         [zb,pb,kb] = butter(2,2*pi*6,'s');
         [a,b] = zp2tf(zb,pb,kb);
 
         dados_filtro(1,:) = data(:,1);
         dados_filtro(2,:) = data(:,7);
         dado = dados_filtro;
         save('untitled.mat','dado');
         simout = sim('ButterWorth_filter','SrcWorkspace','current');
         T2 = simout.simout;
         T2 = T2(1:end-1);
 
         [zb,pb,kb] = butter(2,2*pi*6,'s');
         [a,b] = zp2tf(zb,pb,kb);
         dados_filtro(1,:) = data(:,1);
         dados_filtro(2,:) = data(:,10);
         dado = dados_filtro;
         save('untitled.mat','dado');
         simout = sim('ButterWorth_filter','SrcWorkspace','current');
         T3 = simout.simout;
         T3 = T3(1:end-1);

         [zb,pb,kb] = butter(2,2*pi*6,'s');
         [a,b] = zp2tf(zb,pb,kb);
         dados_filtro(1,:) = data(:,1);
         dados_filtro(2,:) = data(:,13);
         dado = dados_filtro;
         save('untitled.mat','dado');
         simout = sim('ButterWorth_filter','SrcWorkspace','current');
         T4 = simout.simout;
         T4 = T4(1:end-1);
         
         [zb,pb,kb] = butter(2,2*pi*6,'s');
         [a,b] = zp2tf(zb,pb,kb);
         dados_filtro(1,:) = data(:,1);
         dados_filtro(2,:) = data(:,16);
         dado = dados_filtro;
         save('untitled.mat','dado');
         simout = sim('ButterWorth_filter','SrcWorkspace','current');
         T5 = simout.simout;
         T5 = T5(1:end-1);
         
         [zb,pb,kb] = butter(2,2*pi*10,'s');
         [a,b] = zp2tf(zb,pb,kb);
         dados_filtro(1,:) = data(:,1);
         dados_filtro(2,:) = data(:,19);
         dado = dados_filtro;
         save('untitled.mat','dado');
         simout = sim('ButterWorth_filter','SrcWorkspace','current');
         T6 = simout.simout;
         T6 = T6(1:end-1);
         
         delete('untitled.mat');
         data = [time,Pos1,Vel1,T1,Pos2,Vel2,T2,Pos3,Vel3,T3,Pos4,Vel4,T4,Pos5,Vel5,T5,Pos6,Vel6,T6];
     end
    save (outputpath,'data');
end