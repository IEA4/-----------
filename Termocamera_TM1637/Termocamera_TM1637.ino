// GND --- термистор --- A6 --- 10к --- 5V

#define Max_set_temp 90       // максимально возможная выставляемая температура
#define Min_set_temp 28       // минимально ....

#define TIME_DISPLAY 15000    // длительность отображения значения на дисплее

#define NTC_PIN A6            // NTC термистор
#define POTEN_PIN A1          // пин регулятора: выставление нужных оборотов в секунду

#define ZERO_PIN 2            // пин детектора нуля
#define DIMMER_PIN 4          // управляющий пин симистора
#define btnPin 3              // кнопка вызова отображения на семисегментнике

#define DIO_PIN A4            // сигнальные пины TM1637
#define CLK_PIN A5            //        ...

#define period 1              // период расчетов и изменений (мс) в ПИД-регуляторе

#include <Arduino.h>
#include <GyverSegment.h>
Disp1637_4 disp(DIO_PIN, CLK_PIN);

#include "GyverPID.h"         // библиотека ПИД-регулятора      // подробнее о библиотеке https://alexgyver.ru/gyverpid/
GyverPID pid(3000, 500, 0);   // назначение коэфов Kp, Ki, Kd   (25, 100, 0.15)      (25, 50, 0.1)    (30 50 0.1)   (30 45 0.1)   https://alexgyver.ru/lessons/pid/

#include <GyverNTC.h>                            //  GND --- термистор --- A6 --- 10к --- 5V
GyverNTC therm(NTC_PIN, 9450, 3491, 22, 9940);        //  (пин, R* термистора, бета-коэф, температура определения R* в Цельсиях, сопротивление резистора от A6 к 5V)

#include <FastDefFunc.h>                         // библиотека для убыстрения функций: pinMode, digitalWrite, ...Read, analogWrite, ...Read

#include <GyverTimers.h>                         // библиотека таймера         // https://alexgyver.ru/gyvertimers/

#include "GyverFilters.h"                        // библиотека  фильтров       // https://alexgyver.ru/gyverfilters/
GFilterRA filt_pot;                              // создание объекта класса фильтра данных с потенциометра (регулятора)  // https://alexgyver.ru/lessons/filters/
GMedian<16, float> temp_filtr;     //медианный фильтр температуры 16 значений, для вывода текущей температуры на семисегментник

#include <VirtualButton.h>
VButton btn;                      //экземпляр кнопки

volatile unsigned int dimmer;     // переменная диммера текущая(мкс)
volatile unsigned int lastDim;    //                ... предыдущая (мкс)

void setup(){
    pinMode(btnPin, INPUT_PULLUP);        // кнопка притянута к земле, резистор не нужен

    pid.setLimits(500, 9500);       // ограничение выходного сигнала (по умолчанию 0-255)
    pid.setDirection(REVERSE);      // обратное воздействие: увеличению соот. уменьшение (из-за того, что 9500 соответствует минимуму, 500 - макс. открытия симмистора)
    pid.setDt(period);              // временной шаг расчёта функции ПИД-регулятора

    pid.integral = 9500;            // ввиду REVERSE минимальное значение 9500

    filt_pot.setCoef(0.1);          // фильтр для потенциометра //резкость фильтрации (0.00 -- 1.00), чем выше, тем больше скачков

    Timer2.enableISR(CHANNEL_A);    // подкл-но стандартное прерывание, канал B, без сдига фаз, частота ШИМ изменена на D11

    attachInterrupt(digitalPinToInterrupt(ZERO_PIN), isr, RISING); // функция прерывания вызывается по смене сигнала с 0 на 1

    delay(5);                           // БЕЗ НЕГО ПРИ ПОДКЛЮЧЕННОМ К ДИММЕРУ 230В В МОМЕНТ ВКЛ ПИТАНИЯ НА МК НА ВЫХОДАХ ЕГО УПРАВЛЯЮЩИХ ПИНОВ ПРОСКАКИВАЕТ ИНОГДА HIGH
    pinModeFast(DIMMER_PIN, OUTPUT);    // при OUTPUT на пине по умолчанию 0

    disp.printRight(true);  // печатать справа
    disp.setCursorEnd();    // курсор в конец
    disp.brightness(2);     // яркость дисплея 0..7
}

void loop(){
  button();       //функция обработки нажатия кнопки

  while (int(filt_pot.filteredTime(analogReadFast(POTEN_PIN))) > 2)    {     // >2, чтобы отсеч шумы,
      button();
      PID();          // ПИД-регулирование согласно данным с потенциометра
  }
  pid.integral = 9500; // для плавного начала после первого цикла
  dimmer = 9500;       // чтобы выключить симистор
}

// функция ПИД-регулятора
void PID(){
  static unsigned long tmr_pid;           // для отсчёта времени при ПИД-регулировании
  if (millis() - tmr_pid >= period)    {                                                                                          // производим ПИД-регулирование каждую 1мс
        pid.setpoint = map(filt_pot.filteredTime(analogReadFast(POTEN_PIN)), 0, 1023, Min_set_temp, Max_set_temp); // берутся с пот-ра выставленное значение оборотов двигателя, сразу ф-ые
        pid.input = therm.getTemp();                                                           // получаем новые данные
        pid.getResult();                                                                       // производится расчёт, определяется насколько умень/увел. выходной сигнал для соот. данным с потенциометра
        dimmer = int(expRAA(pid.output));                                                      // на управляющее устройство даётся расчитанный сигнал
        tmr_pid = millis();
    }
}

//функция обработки нажатия кнопки
void button(){
  static long t_pot;
  static bool f_click, f_step;     // логические переменные для кнопки
  static bool power_state = 0;

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

  if(!power_state){
    disp.power(true);
    power_state = 1;
  }

  if (f_click){                         // при клике кнопкой
    static float filT;
    filT = round(10 * temp_filtr.filtered(therm.getTemp()))/10.0; // получаем температуру, фильтруем, округляем до десятых
    disp.clear();
    disp.print(filT,1);
  }
  if (f_step){                          // при удержании кнопки
    static int pot, pot_1;              // переменные для определения налиячия изменения значения
    pot = map(filt_pot.filteredTime(analogReadFast(POTEN_PIN)), 0, 1023, Min_set_temp, Max_set_temp); //получаем значение с ротенциометра, фильтруем, конформно отображаем на другой диапазон
    if (pot != pot_1){                  // если значения изменились
      pot_1 = pot;
      t_pot = millis();                 //  сбрасываем таймер
    }
    disp.clear();
    disp.print(pot_1);                                   //  ... и выводим на семисегментник
  }

  disp.update();
  disp.delay(100);

  if (f_click && btn.timeout(TIME_DISPLAY)){                     // если после клика прошло более 15 секунд, выключаем семисегментник
    f_click = 0;
    disp.clear();
    disp.power(false);
    power_state = 0;
  }

  if(f_step && millis() - t_pot > TIME_DISPLAY){               // если значение на потенциометре не меняется в течении 15 секунд, выключаем семисегментник
    f_step = 0;
    disp.clear();
    disp.power(false);
    power_state = 0;
  }
}

// функция прерывания детектора нуля
void isr(){
    digitalWriteFast(DIMMER_PIN, 0); // выключаем симистор
    if (lastDim != dimmer)
        Timer2.setPeriod(lastDim = dimmer); // если значение изменилось, устанавливаем новый период
    else
        Timer2.restart(); // если нет, то просто перезапускаем со старым//перезапустить таймер (сбросить счётчик) с новым периодом
}

// прерывание таймера диммера
ISR(TIMER2_A){
    digitalWrite(DIMMER_PIN, 1); // включаем симистор
    Timer2.stop();               // останавливаем таймер
}

// бегущее среднее с адаптивным коэффициентом
float expRAA(float newVal){
    static float filVal = 0;
    float k;
    if (abs(newVal - filVal) > 9000)
        k = 0.1; // резкость фильтра зависит от модуля разности значений
    else
        k = 0.05;

    filVal += (newVal - filVal) * k;
    return filVal;
}