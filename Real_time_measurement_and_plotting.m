clc; close all; clear variables;

s = serialport("COM4",9600); %konfiguracja portu szeregowego
configureTerminator(s,'CR');% nawiazanie komunikacji
y=[0]; %utworzenie wektora pomiarow
t=[0]; %utwrzoenie wektora czasu
while 1
    text=readline(s); %odczyt wartosci
    X = str2double(text); %zamiana z typu string na double
    y(end+1)=X; %aktualizacja wektora wartosci 
    t(end+1)=t(end)+0.1; %aktualizacja wektora czasu 
    plot(t,y,'b') %wyswietlenie wykresu
    min=length(t)/10-20;
    if min<0
        min=0;
    end
    title('Pomiar odleglosci w czasie rzeczywistym')
    ylim([0 40]); %ograniczenie osi y
    xlim([min min+20]); %ograniczenie osi x do ostatnich 20 sekund
    xlabel('Czas t [s]')
    ylabel('Odleglosc [cm]')
end