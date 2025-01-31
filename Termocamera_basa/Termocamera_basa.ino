/*
Для каждого типа резистивного нагревателя свои коэффициенты ПИД-регулирования
В строке GyverNTC therm(NTC_PIN, 10000, 3490, 22, 11640) ввести свои данные
*/

#define ZERO_PIN 2            // пин детектора нуля
#define DIMMER_PIN 4          // управляющий пин симистора

#define NTC_PIN A6            // пин NTC-термистора
#define POTEN_PIN A1          // пин регулятора: выставление нужных оборотов в секунду

#define period 1              // период расчетов и изменений (мс) в ПИД-регуляторе

#include "GyverPID.h"         // библиотека ПИД-регулятора      // подробнее о библиотеке https://alexgyver.ru/gyverpid/
GyverPID pid(3000, 500, 0);   // назначение коэфов Kp, Ki, Kd   //  https://alexgyver.ru/lessons/pid/

#include <GyverNTC.h>                            //  GND --- термистор --- A6 --- 10к --- 5V // бета-коэф 3950 несоот-вует опыту, подходит 3490
GyverNTC therm(NTC_PIN, 10000, 3490, 22, 11640);       //  (пин, R* термистора при 20град.С, бета-коэф, температура определения R*, сопротивление резистор от NTC_PIN к 5V)

#include <FastDefFunc.h>              // библиотека для убыстрения функций: pinMode, digitalWrite, ...Read, analogWrite, ...Read

#include <GyverTimers.h>              // библиотека таймера         // https://alexgyver.ru/gyvertimers/

#include "GyverFilters.h"             // библиотека  фильтров       // https://alexgyver.ru/gyverfilters/
GFilterRA filt_pot;                   // создание объекта класса фильтра данных с потенциометра (регулятора)  // https://alexgyver.ru/lessons/filters/

volatile unsigned int dimmer;         // переменная диммера текущая(мкс)
volatile unsigned int lastDim;        //                ... предыдущая (мкс)

unsigned long tmr_pid;                // для отсчёта времени при ПИД-регулировании

void setup()
{
    //Serial.begin(115200);             // скорость взаимодействи в мониторе порта, 9600 -- слишком мало
    //Serial.flush();                   // ждём окончания передачи предыдущих данных

    //Serial.println("in, set, dim");   // для отображения в плоттере

    pid.setLimits(500, 9500);           // ограничение выходного сигнала (по умолчанию 0-255)
    pid.setDirection(REVERSE);          // обратное воздействие: увеличению соот. уменьшение (из-за того, что 9500 соответсвует минимуму, 500 - макс. открытия симмистора)
    pid.setDt(period);                  // временной шаг расчёта функции ПИД-регулятора

    pid.integral = 9500;                // ввиду REVERSE минимальное значение 9500

    filt_pot.setCoef(0.1);              // фильтр для потенциометра //резкость фильтрации (0.00 -- 1.00), чем выше, тем больше скачков

    Timer2.enableISR(CHANNEL_A);        // подкл-но стандартное прерывание, канал B, без сдига фаз, частота ШИМ изменена на D11

    attachInterrupt(digitalPinToInterrupt(ZERO_PIN), isr, RISING);    // функция прерывания вызывается по смене сигнала с 0 на 1

    delay(5);                                // БЕЗ НЕГО ПРИ ПОДКЛЮЧЕННОМ К ДИММЕРУ 230В В МОМЕНТ ВКЛ ПИТАНИЯ НА МК НА ВЫХОДАХ ЕГО УПРАВЛЯЮЩИХ ПИНОВ ПРОСКАКИВАЕТ ИНОГДА HIGH
    pinModeFast(DIMMER_PIN, OUTPUT);         // при OUTPUT на пине по умолчанию 0
}

void loop()
{
    while (int(filt_pot.filteredTime(analogReadFast(POTEN_PIN))) > 2)   // ПИД-регулирование пока ручка повёрнута ручка регулятора
    {
        PID();                  // ПИД-регулирование согласно данным с потенциометра
        //serial_print();         // вывод желаемых данных в Serial_plotter
    }
    pid.integral = 9500;        // для плавного начала после первого цикла
    dimmer = 9500;              // чтобы выключить симистор
}

// функция ПИД-регулятора
void PID()
{
  if (millis() - tmr_pid >= period)               // производим ПИД-регулирование каждую миллисекунду
    {
        pid.setpoint = map(filt_pot.filteredTime(analogReadFast(POTEN_PIN)), 0, 1024, 80, 20); // берутся с пот-ра выставленное значение температуры, сразу ф-ые
        pid.input = therm.getTemp();              // получаем новые данные
        pid.getResult();                          // производится расчёт, определяется насколько умень/увел. выходной сигнал для соот. данным с потенциометра
        dimmer = int(expRAA(pid.output));         // на управляющее устройство даётся расчитанный сигнал
        tmr_pid = millis();
    }
}

// вывод данных в Serial_plotter
void serial_print()
{
    Serial.print(pid.input);      // получаемое значение
    Serial.print(',');
    // Serial.print(pid.output); Serial.print(',');
    Serial.println(pid.setpoint); // Serial.print(',');    //   ... выставленных потенциометром
                                  // Serial.println(dimmer);
}

// функция прерывания детектора нуля
void isr()
{
    digitalWriteFast(DIMMER_PIN, 0);        // выключаем симистор
    if (lastDim != dimmer)
        Timer2.setPeriod(lastDim = dimmer); // если значение изменилось, устанавливаем новый период
    else
        Timer2.restart();                   // если нет, то просто перезапускаем со старым//перезапустить таймер (сбросить счётчик) с новым периодом
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
        k = 1; // резкость фильтра зависит от модуля разности значений
    else
        k = 0.1;

    filVal += (newVal - filVal) * k;
    return filVal;
}