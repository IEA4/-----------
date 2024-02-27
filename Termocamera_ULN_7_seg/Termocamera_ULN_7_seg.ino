// GND --- термистор --- A7 --- 10к --- 5V

//Выходы сегментов
#define A  12
#define B  10      // если покажет ошибку в этой строчке, прописать char B = 10;
#define C  8
#define D  A0
#define E  A3
#define F  9
#define G  A2
#define Dp A1

//Выходы транзисторных ключей
#define R3  7
#define R2  6
#define R1  5

#define ZERO_PIN 2            // пин детектора нуля
#define DIMMER_PIN 4          // управляющий пин симистора
#define btnPin 3              // кнопка вызова отображения на семисегментнике

#define NTC_PIN A6            // пин NTC-термистора
#define POTEN_PIN A1          // пин регулятора: выставление нужных оборотов в секунду

#define period 1              // период расчетов и изменений (мс) в ПИД-регуляторе

#include "GyverPID.h"         // библиотека ПИД-регулятора      // подробнее о библиотеке https://alexgyver.ru/gyverpid/
GyverPID pid(3000, 500, 0);   // назначение коэфов Kp, Ki, Kd   (25, 100, 0.15)      (25, 50, 0.1)    (30 50 0.1)   (30 45 0.1)   https://alexgyver.ru/lessons/pid/

#include <GyverNTC.h>                            //  GND --- термистор --- A6 --- 10к --- 5V / бета-коэф 3950 несоответствует опыту, подходит 3490
GyverNTC therm(NTC_PIN, 10000, 3490, 22, 11640);       //  (пин, R* термистора при 20град.С, бета-коэф, температура определения R*, сопротивление резистор от NTC_PIN к 5V)

#include <FastDefFunc.h>                         // библиотека для убыстрения функций: pinMode, digitalWrite, ...Read, analogWrite, ...Read

#include <GyverTimers.h>                         // библиотека таймера         // https://alexgyver.ru/gyvertimers/

#include "GyverFilters.h"                        // библиотека  фильтров       // https://alexgyver.ru/gyverfilters/
GFilterRA filt_pot;                              // создание объекта класса фильтра данных с потенциометра (регулятора)  // https://alexgyver.ru/lessons/filters/
GMedian<16, float> temp_filtr;     //медианный фильтр температуры 16 значений, для вывода текущей температуры на семисегментник

#include <VirtualButton.h>
VButton btn;                      //экземпляр кнопки

volatile unsigned int dimmer;  // переменная диммера текущая(мкс)
volatile unsigned int lastDim; //                ... предыдущая (мкс)

unsigned long tmr_pid; // для отсчёта времени при ПИД-регулировании

String filT, pot, pot_1;  // строковые переменные для измеренной температуры и высттавленных потенциометром значений
bool f_click, f_step;     // логические переменные для кнопки

unsigned long t_s, t_pot;  // переменные времени для включения сегмента и определения периода изменения потенциометра
void setup()
{
    pinModeFast(A, OUTPUT);
    pinModeFast(B, OUTPUT);
    pinModeFast(C, OUTPUT);
    pinModeFast(D, OUTPUT);
    pinModeFast(E, OUTPUT);
    pinModeFast(F, OUTPUT);
    pinModeFast(G, OUTPUT);
    pinModeFast(Dp, OUTPUT);

    pinModeFast(R1, OUTPUT);
    pinModeFast(R2, OUTPUT);
    pinModeFast(R3, OUTPUT);

    pinMode(btnPin, INPUT_PULLUP);        // кнопка притянута к земле, резистор не нужен

    pid.setLimits(500, 9500);  // ограничение выходного сигнала (по умолчанию 0-255)
    pid.setDirection(REVERSE); // обратное воздействие: увеличению соот. уменьшение (из-за того, что 9500 соответсвует минимуму, 500 - макс. открытия симмистора)
    pid.setDt(period);         // временной шаг расчёта функции ПИД-регулятора

    pid.integral = 9500; // ввиду REVERSE минимальное значение 9500

    filt_pot.setCoef(0.1); // фильтр для потенциометра //резкость фильтрации (0.00 -- 1.00), чем выше, тем больше скачков

    Timer2.enableISR(CHANNEL_A); // подкл-но стандартное прерывание, канал B, без сдига фаз, частота ШИМ изменена на D11

    attachInterrupt(digitalPinToInterrupt(ZERO_PIN), isr, RISING); // функция прерывания вызывается по смене сигнала с 0 на 1

    delay(5);                        // БЕЗ НЕГО ПРИ ПОДКЛЮЧЕННОМ К ДИММЕРУ 230В В МОМЕНТ ВКЛ ПИТАНИЯ НА МК НА ВЫХОДАХ ЕГО УПРАВЛЯЮЩИХ ПИНОВ ПРОСКАКИВАЕТ ИНОГДА HIGH
    pinModeFast(DIMMER_PIN, OUTPUT); // при OUTPUT на пине по умолчанию 0
}

void loop()
{
    button();       //функция обработки нажатия кнопки

    while (int(filt_pot.filteredTime(analogReadFast(POTEN_PIN))) > 2)
    {
        button();
        PID();          // ПИД-регулирование согласно данным с потенциометра
    }
    pid.integral = 9500; // для плавного начала после первого цикла
    dimmer = 9500;       // чтобы выключить симистор
}

// функция ПИД-регулятора
void PID()
{
  if (millis() - tmr_pid >= period)
    {                                                                                          // производим ПИД-регулирование каждую 1мс
        pid.setpoint = map(filt_pot.filteredTime(analogReadFast(POTEN_PIN)), 0, 1024, 80, 20); // берутся с пот-ра выставленное значение температуры, сразу ф-ые
        pid.input = therm.getTemp();                                                           // получаем новые данные
        pid.getResult();                                                                       // производится расчёт, определяется насколько умень/увел. выходной сигнал для соот. данным с потенциометра
        dimmer = int(expRAA(pid.output));                                                      // на управляющее устройство даётся расчитанный сигнал
        tmr_pid = millis();
    }
}

//функция обработки нажатия кнопки
void button()
{
  btn.poll(!digitalRead(btnPin));
  if (btn.click()){                     //был клик кнопкой
    f_click = 1;
    f_step = 0;
  }
  if (btn.step()){                      //если было удержание кнопки
    f_step = 1;
    f_click = 0;
    t_pot = millis();
  }

  if (f_click){                         // при клике кнопкой
    filT = String((round(10 * temp_filtr.filtered(therm.getTemp())))/10.0); // получаем температуру, фильтруем, округляем до десятых
    black_print(filT);                                                         //  ... и выводим на семисегментник
  }
  if (f_step){                          //при удержании кнопки
    pot = String(map(filt_pot.filteredTime(analogReadFast(POTEN_PIN)), 0, 1024, 80, 20)); //получаем значение с потенциометра, фильтруем, конформно отображаем на другой диапазон
    if (pot != pot_1){
      pot_1 = pot;
      t_pot = millis();
    }
    black_print(pot_1);                                   //  ... и выводим на семисегментник
  }

  if (f_click && btn.timeout(15000)){                     // если после клика прошло более 15 секунд, выключаем семисегментник
     f_click = 0;
     digitalWriteFast(R1, LOW);
     digitalWriteFast(R2, LOW);
     digitalWriteFast(R3, LOW);
  }

  if(f_step && millis() - t_pot > 15000){               // если значение на потенциометре не меняется в течении 15 секунд, выключаем семисегментник
    f_step = 0;
    digitalWriteFast(R1, LOW);
    digitalWriteFast(R2, LOW);
    digitalWriteFast(R3, LOW);
  }
}

// функция прерывания детектора нуля
void isr()
{
    digitalWriteFast(DIMMER_PIN, 0); // выключаем симистор
    if (lastDim != dimmer)
        Timer2.setPeriod(lastDim = dimmer); // если значение изменилось, устанавливаем новый период
    else
        Timer2.restart(); // если нет, то просто перезапускаем со старым//перезапустить таймер (сбросить счётчик) с новым периодом
}

// прерывание таймера диммера
ISR(TIMER2_A)
{
    digitalWrite(DIMMER_PIN, 1); // включаем симистор
    Timer2.stop();               // останавливаем таймер
}

// бегущее среднее с адаптивным коэффициентом
float expRAA(float newVal)
{
    static float filVal = 0;
    float k;
    if (abs(newVal - filVal) > 9000)
        k = 0.1; // резкость фильтра зависит от модуля разности значений
    else
        k = 0.05;

    filVal += (newVal - filVal) * k;
    return filVal;
}

//функция вывода значений на семисегментник
void black_print(String x) {
  switch (x.length()) {         // определяется длина строки
                                //                        ... и в зависиммости от него производится выбор сегментов для задействования
    case 2:                     // для целочисленных чисел больше 9 и меньше 100
      if (millis() - t_s <= 5){
        digitalWriteFast(R3, LOW);                //третий сегмент выкл
        digitalWriteFast(R2, HIGH);               //     ..второй вкл
        digitalWriteFast(R1, LOW);                //     ..третий выкл
        number(x[0] - '0');                       // изображаем первую цифру заданного числа
      }
      else if (millis() - t_s > 5 && millis() - t_s <= 10){
        digitalWriteFast(R3, LOW);
        digitalWriteFast(R2, LOW);
        digitalWriteFast(R1, HIGH);               // изображаем первую цифру заданного числа
        number(x[1] - '0');
      }
      else t_s = millis();                        // чтоб начать заново высвечивать
      break;

    case 5:                                     // для чисел больше 9 и  c плавающей точкой
      if (millis() - t_s <= 5){
        digitalWriteFast(R3, HIGH);
        digitalWriteFast(R2, LOW);
        digitalWriteFast(R1, LOW);
        number(x[0] - '0');
      }
      else if (millis() - t_s > 5 && millis() - t_s <= 10){
        digitalWriteFast(R3, LOW);
        digitalWriteFast(R2, HIGH);
        digitalWriteFast(R1, LOW);
        number(x[1] - '0');
      }
      else if (millis() - t_s > 10 && millis() - t_s <= 15){
        digitalWriteFast(R3, LOW);
        digitalWriteFast(R2, HIGH);
        digitalWriteFast(R1, LOW);
        number(x[2] - '0');
      }
      else if (millis() - t_s > 15 && millis() - t_s <= 20){
        digitalWriteFast(R3, LOW);
        digitalWriteFast(R2, LOW);
        digitalWriteFast(R1, HIGH);
        number(x[3] - '0');
      }
      else t_s = millis();
      break;
  }
}

void number (byte num){
  switch (num) {
    case 0:
       digitalWriteFast(A, HIGH); //цифра ноль
       digitalWriteFast(B, HIGH);
       digitalWriteFast(C, HIGH);
       digitalWriteFast(D, HIGH);
       digitalWriteFast(E, HIGH);
       digitalWriteFast(F, HIGH);
       digitalWriteFast(G, LOW);
       digitalWriteFast(Dp, LOW);
       break;
    case 1:
       digitalWriteFast(A, LOW); //цифра один
       digitalWriteFast(B, HIGH);
       digitalWriteFast(C, HIGH);
       digitalWriteFast(D, LOW);
       digitalWriteFast(E, LOW);
       digitalWriteFast(F, LOW);
       digitalWriteFast(G, LOW);
       digitalWriteFast(Dp, LOW);
       break;
    case 2:
       digitalWriteFast(A, HIGH); //цифра два
       digitalWriteFast(B, HIGH);
       digitalWriteFast(C, LOW);
       digitalWriteFast(D, HIGH);
       digitalWriteFast(E, HIGH);
       digitalWriteFast(F, LOW);
       digitalWriteFast(G, HIGH);
       digitalWriteFast(Dp, LOW);
       break;
    case 3:
       digitalWriteFast(A, HIGH); //цифра три
       digitalWriteFast(B, HIGH);
       digitalWriteFast(C, HIGH);
       digitalWriteFast(D, HIGH);
       digitalWriteFast(E, LOW);
       digitalWriteFast(F, LOW);
       digitalWriteFast(G, HIGH);
       digitalWriteFast(Dp, LOW);
       break;
    case 4:
       digitalWriteFast(A, LOW); //цифра четыре
       digitalWriteFast(B, HIGH);
       digitalWriteFast(C, HIGH);
       digitalWriteFast(D, LOW);
       digitalWriteFast(E, LOW);
       digitalWriteFast(F, HIGH);
       digitalWriteFast(G, HIGH);
       digitalWriteFast(Dp, LOW);
       break;
    case 5:
       digitalWriteFast(A, HIGH); //цифра пять
       digitalWriteFast(B, LOW);
       digitalWriteFast(C, HIGH);
       digitalWriteFast(D, HIGH);
       digitalWriteFast(E, LOW);
       digitalWriteFast(F, HIGH);
       digitalWriteFast(G, HIGH);
       digitalWriteFast(Dp, LOW);
       break;
    case 6:
       digitalWriteFast(A, HIGH); //цифра шесть
       digitalWriteFast(B, LOW);
       digitalWriteFast(C, HIGH);
       digitalWriteFast(D, HIGH);
       digitalWriteFast(E, HIGH);
       digitalWriteFast(F, HIGH);
       digitalWriteFast(G, HIGH);
       digitalWriteFast(Dp, LOW);
       break;
    case 7:
       digitalWriteFast(A, HIGH); //цифра семь
       digitalWriteFast(B, HIGH);
       digitalWriteFast(C, HIGH);
       digitalWriteFast(D, LOW);
       digitalWriteFast(E, LOW);
       digitalWriteFast(F, LOW);
       digitalWriteFast(G, LOW);
       digitalWriteFast(Dp, LOW);
       break;
    case 8:
       digitalWriteFast(A, HIGH); //цифра восемь
       digitalWriteFast(B, HIGH);
       digitalWriteFast(C, HIGH);
       digitalWriteFast(D, HIGH);
       digitalWriteFast(E, HIGH);
       digitalWriteFast(F, HIGH);
       digitalWriteFast(G, HIGH);
       digitalWriteFast(Dp, LOW);
       break;
    case 9:
       digitalWriteFast(A, HIGH); //цифра девять
       digitalWriteFast(B, HIGH);
       digitalWriteFast(C, HIGH);
       digitalWriteFast(D, HIGH);
       digitalWriteFast(E, LOW);
       digitalWriteFast(F, HIGH);
       digitalWriteFast(G, HIGH);
       digitalWriteFast(Dp, LOW);
       break;
   default:
       digitalWriteFast(A, LOW); //точка
       digitalWriteFast(B, LOW);
       digitalWriteFast(C, LOW);
       digitalWriteFast(D, LOW);
       digitalWriteFast(E, LOW);
       digitalWriteFast(F, LOW);
       digitalWriteFast(G, LOW);
       digitalWriteFast(Dp, HIGH);
       break;
  }
}
