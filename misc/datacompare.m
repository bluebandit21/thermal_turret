stm32 = readtable('STM32.csv');
arduino = readtable('Arduino.csv');


stTimes = table2array(stm32(:,1));
stClks = table2array(stm32(:,4));
stDats = table2array(stm32(:,3));

arTimes = table2array(arduino(:,1));
arClks = table2array(arduino(:,4));
arDats = table2array(arduino(:,3));

stTime = zeros(1,numel(stTimes));
stClk = zeros(1,numel(stClks));
stDat = zeros(1,numel(stDats));

arTime = zeros(1,numel(arTimes));
arClk = zeros(1,numel(arClks));
arDat = zeros(1,numel(arDats));

%turn time line AR-TI
for i = 1:numel(arDats)
    arTime(i) = arTimes(i,1);
end
%turn clock line AR-Clk
for i = 1:numel(arDats)
    arClk(i) = arClks(i,1);
end
%turn time line ST-TI
for i = 1:numel(arDats)
    stTime(i) = stTimes(i,1);
end
%turn clock line ST-Clk
for i = 1:numel(arDats)
    stClk(i) = stClks(i,1);
end



%Add 2 offset to data line AR-DAT
for i = 1:numel(arDats)
    arDat(i) = arDats(i,1) + 2;
end

%Add 2 offset to data line ST-DAT
for i = 1:numel(arDats)
    stDat(i) = stDats(i,1) + 2;
end

p = plot(arTime,arClk, '-b');
hold off;

p2 = plot(arTime, arDat, '-k');

p3 = plot(stTime, stClk, '-g');
p4 = plot(stTime, stDat, '-r');




