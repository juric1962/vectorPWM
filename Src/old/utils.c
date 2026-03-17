#include "utils.h"
#include "string.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "scenario_softAP.h"
//using namespace std;

//#define _CRT_SECURE_NO_WARNINGS

char HTMLString[MAX_SIZE_HTML_PACKET];
// Счетчик элементов в массиве htmlList
int CountItemInHtmlList = 0;
ThtmlItem htmlList[MAX_HTML_ITEM];

char SectionName[SECTION_SIZE];
extern char comand_WiFi;
extern volatile uint8_t rinatTimer;

const char ArrSection[MAX_ARRSECTION][SECTION_SIZE] = {
   "Дата и время",
   "CFG ЭТР",
   "CFG OVERLOAD",
   "CFG Перекос тока",
   "CFG HIGH DC_BUS",
   "CFG Холостой Ход",
   "CFG LOW DC-BUS",
   "CFG Температура",
   "CFG упр ПЧ",
   "CFG ПИД",
   "CFG ПЧ",
   "CFG ТИТ",
   "CFG АО",
   "ТИИ",
   "CFG MOTOR",
   "CFG Fmin",
   "CFG Fmid",
   "CFG Fmax",
   "CFG Fmid point",
   "CFG старт-частота(Гц)",
   "CFG разгон(Гц/c)",
   "CFG торможение(Гц/с)",
   "CFG S х-ка разгона(с)",
   "CFG S х-ка торможения(с)",
   "CFG Номер U(F)",
   "CFG задание: 0-MB,1-ТИТ1,2-ТИТ2",
   "CFG контекст",
   "счетчики событий",
   "CFG ARH(привязка входа ТС (0-15))",
   "CFG ARH(интервал, мин)",
   "CFG ARH",
   "WATTH(кВт*час)",
   "Время UNIX (секунды с 01_01_2000г)"
};



const TParamItem ParamItem[MAX_PARAM_ITEM] = {
   "Секунды(0-59)", 10, UINT, 255, 1, false, 0,
   "Минуты(0-59)", 11, UINT, 255, 1, false, 0,
   "Часы(0-23)", 12, UINT, 255, 1, false, 0,
   "День(1-31)", 13, UINT, 255, 1, false, 0,
   "Месяц(1-12)", 14, UINT, 255, 1, false, 0,
   "Год(+2000)", 15, UINT, 255, 1, false, 0,
   "Вторая точка частотной корректировки тока(Гц)", 20, REAL, 255, 1, false, 1,
   "Третья точка частотной корректировки тока(Гц)", 22, REAL, 255, 1, false, 1,
   "Первый поправочный коэффициент тока(%)", 24, REAL, 255, 1, false, 1,
   "Второй поправочный коэффициент тока(%)", 26, REAL, 255, 1, false, 1,
   "Третий поправочный коэффициент тока(%)", 28, REAL, 255, 1, false, 1,
   "Четвертый поправочный коэффициент тока(%)", 30, REAL, 255, 1, false, 1,
   "Аварийная уставка относительно Iном(%)", 32, UINT, 255, 1, false, 1,
   "Процент относительно аварийного счетчика ЭТР, позволяющий возобновить работу", 33, UINT, 255, 1, false, 1,
   "Аварийная уставка по времени(с)", 34, UINT, 255, 1, false, 1,
   "Наличие внешнего охлаждения: 0-нет, 1-есть ", 35, UINT, 255, 1, false, 1,
   "Состояние защиты: 0-выкл, 1-вкл, 2-сброс счетчика ", 36, UINT, 255, 1, false, 1,
   "Мгновенное значение(%) от ном", 50, UINT, 255, 1, false, 2,
   "Кол-во отсчетов для усреднения мгновенных токов", 51, UINT, 255, 1, false, 2,
   "Дейстующее значение тока(%) от ном", 52, UINT, 255, 1, false, 2,
   "Время удержания действующего значения тока(мс)", 53, UINT, 255, 1, false, 2,
   "Max значение счетчика АПВ", 54, UINT, 255, 1, false, 2,
   "Время декрементирования счетчика АПВ(мин)", 55, UINT, 255, 1, false, 2,
   "Величина тока обрыва фазы(%) от ном", 56, UINT, 255, 1, false, 3,
   "Время удержания обрыва фазы(мс)", 57, UINT, 255, 1, false, 3,
   "Дисбаланс тока(%) от ном", 58, UINT, 255, 1, false, 3,
   "Время удержания дисбаланса тока(мс)", 59, UINT, 255, 1, false, 3,
   "Max значение счетчика АПВ", 60, UINT, 255, 1, false, 3,
   "Время декрементирования счетчика АПВ(мин)", 61, UINT, 255, 1, false, 3,
   "Max значение счетчика АПВ", 62, UINT, 255, 1, false, 4,
   "Время декрементирования счетчика АПВ(мин)", 63, UINT, 255, 1, false, 4,
   "Активная мощность(Вт)", 64, UINT, 255, 1, false, 5,
   "Время удержания(с)", 65, UINT, 255, 1, false, 5,
   "Max значение счетчика АПВ", 66, UINT, 255, 1, false, 5,
   "Время декрементирования счетчика АПВ(мин)", 67, UINT, 255, 1, false, 5,
   "Max значение счетчика АПВ", 68, UINT, 255, 1, false, 1,
   "Время декрементирования счетчика АПВ(мин)", 69, UINT, 255, 1, false, 1,
   "Max значение счетчика АПВ", 70, UINT, 255, 1, false, 6,
   "Время декрементирования счетчика АПВ(мин)", 71, UINT, 255, 1, false, 6,
   "Разрешение работы в этом режиме(>0-разрешено)", 72, UINT, 255, 1, false, 6,
   "Напряжение для перехода в режим РФЧ(фиксированной частоты)(В)", 73, UINT, 255, 1, false, 6,
   "Время удержания напряжения(мс)", 74, UINT, 255, 1, false, 6,
   "Размах 100Гц гармоники для перехода в РФЧ(В)", 75, UINT, 255, 1, false, 6,
   "Время удержания размаха 100Гц гармоники(мс)", 76, UINT, 255, 1, false, 6,
   "Max частота в режиме РФЧ(Гц)", 77, UINT, 255, 1, false, 6,
   "Кол-во попыток выхода из РФЧ(0-сразу переход в аварию)", 78, UINT, 255, 1, false, 6,
   "Периодичность выхода из РФЧ(0-не выходить)(мин)", 79, UINT, 255, 1, false, 6,
   "Период сброса счетчика попыток выхода из РФЧ(0-не сбрасывать)(мин) ", 80, UINT, 255, 1, false, 6,
   "Max значение счетчика АПВ", 81, UINT, 255, 1, false, 7,
   "Время декрементирования счетчика АПВ(мин)", 82, UINT, 255, 1, false, 7,
   "Аварийная температура отсека конденсаторов(0С)", 83, UINT, 255, 1, false, 7,
   "Температура конденсаторов, допускающая возобновление работы(0С)", 84, UINT, 255, 1, false, 7,
   "Аварийная температура радиатора(0С)", 85, UINT, 255, 1, false, 7,
   "Температура радиатора, допускающая возобновление работы(0С)", 86, UINT, 255, 1, false, 7,
   "Частота для установки", 100, REAL, 255, 1, false, 8,
   "Стабилизация напряжения(>0-вкл)", 102, UINT, 255, 1, false, 8,
   "Направление вращения(0-по часовой,1-против)", 103, UINT, 255, 1, false, 8,
   "Останов двигателя по модбас в режиме ЧРЭП(>0)", 104, UINT, 255, 1, false, 8,
   "Задержка на включении после аварии в режимах ЧРЭП и КП(с)", 105, UINT, 255, 1, false, 8,
   "Задержка перед пуском в режиме КП(с)", 106, UINT, 255, 1, false, 8,
   "Сброс аварий(>0)", 107, UINT, 255, 1, false, 8,
   "Автономный режим работы: 0-ЧРЭП, 1-КП", 108, UINT, 255, 1, false, 8,
   "Температура вкл вентиляторов и ТУ3(0С)", 110, UINT, 255, 1, false, 8,
   "Гистерезис температуры вкл/выкл вентиляторов и ТУ3(0С)", 111, UINT, 255, 1, false, 8,
   "Предельно допустимое увеличение частоты(Гц)", 112, UINT, 255, 1, false, 8,
   "Величина стабилизации мощности (Вт)", 114, REAL, 255, 1, false, 9,
   "Пропорциональный коэффициент", 116, REAL, 255, 1, false, 9,
   "Интегральный коэффициент", 118, REAL, 255, 1, false, 9,
   "Дифференциальный коэффициент", 120, REAL, 255, 1, false, 9,
   "Ограничение на Max изменение фазы напряжения", 122, REAL, 255, 1, false, 9,
   "Нижний предел отключения ПИД-регулятора(Вт)", 124, UINT, 255, 1, false, 9,
   "Верхний предел включения ПИД-регулятора(Вт)", 125, UINT, 255, 1, false, 9,
   "Стартовый ток увеличения частоты статора(А)", 126, UINT, 255, 1, false, 8,
   "Стартовый ток удержания частоты статора(А)", 127, UINT, 255, 1, false, 8,
   "Время удержания стартовой частоты статора(>0-вкл)(мс)", 128, UINT, 255, 1, false, 8,
   "Величина игнорируемой входной ошибки(Вт)", 129, UINT, 255, 1, false, 9,
   "Действующее напряжение «подхвата» вращающегося ротора(В)", 130, UINT, 255, 1, false, 10,
   "Разрешение «подхвата» вращающегося ротора(>0-вкл)", 131, UINT, 255, 1, false, 10,
   "Min частота, соотв min напряжению ТИТ1(Гц)", 140, UINT, 255, 1, false, 11,
   "Min частота, соотв min напряжению ТИТ2(Гц)", 141, UINT, 255, 1, false, 11,
   "Max частота, соотв max напряжению ТИТ1(Гц)", 142, UINT, 255, 1, false, 11,
   "Max частота, соотв max напряжению ТИТ2(Гц)", 143, UINT, 255, 1, false, 11,
   "Min напряжение ТИТ1(мВ)", 144, UINT, 255, 1, false, 11,
   "Min напряжение ТИТ2(мВ)", 145, UINT, 255, 1, false, 11,
   "Max напряжение ТИТ1(мВ)", 146, UINT, 255, 1, false, 11,
   "Max напряжение ТИТ2(мВ)", 147, UINT, 255, 1, false, 11,
   "Время усреднения ТИТ1(мс)", 148, UINT, 255, 1, false, 11,
   "Время усреднения ТИТ2(мс)", 149, UINT, 255, 1, false, 11,
   "Время усреднения ТИТ3(мс)", 150, UINT, 255, 1, false, 11,
   "Время усреднения ТИТ4(мс)", 151, UINT, 255, 1, false, 11,
   "ДжиттерРазница ТИТ1(мВ)", 152, UINT, 255, 1, false, 11,
   "ДжиттерРазница ТИТ2(мВ)", 153, UINT, 255, 1, false, 11,
   "Чувствительность ТИТ1(в 0,01 Гц)", 154, UINT, 255, 1, false, 11,
   "Чувствительность ТИТ2(в 0,01 Гц)", 155, UINT, 255, 1, false, 11,
   "Min значение измеряемой величины, соотв min напряжению AO1(мВ)", 160, REAL, 255, 1, false, 12,
   "Min значение измеряемой величины, соотв min напряжению AO2(мВ)", 162, REAL, 255, 1, false, 12,
   "Max значение измеряемой величины, соотв max напряжению AO1(мВ)", 164, REAL, 255, 1, false, 12,
   "Max значение измеряемой величины, соотв max напряжению AO2(мВ)", 166, REAL, 255, 1, false, 12,
   "Min напряжение AO1(мВ)", 168, UINT, 255, 1, false, 12,
   "Min напряжение AO2(мВ)", 169, UINT, 255, 1, false, 12,
   "Max напряжение AO1(мВ)", 170, UINT, 255, 1, false, 12,
   "Max напряжение AO2(мВ)", 171, UINT, 255, 1, false, 12,
   "Источник сигнала AO1: 0-выкл, 1-Fвых, 2-об/мин, 3-Pакт, 4-текущий момент", 172, UINT, 255, 1, false, 12,
   "Источник сигнала AO2: 0-выкл, 1-Fвых, 2-об/мин, 3-Pакт, 4-текущий момент", 173, UINT, 255, 1, false, 12,
   "ТИИ1", 180, UINT, 255, 1, false, 13,
   "ТИИ2", 181, UINT, 255, 1, false, 13,
   "ТИИ3", 182, UINT, 255, 1, false, 13,
   "ТИИ4", 183, UINT, 255, 1, false, 13,
   "ТИИ5", 184, UINT, 255, 1, false, 13,
   "ТИИ6", 185, UINT, 255, 1, false, 13,
   "ТИИ7", 186, UINT, 255, 1, false, 13,
   "ТИИ8", 187, UINT, 255, 1, false, 13,
   "ТИИ9", 188, UINT, 255, 1, false, 13,
   "ТИИ10", 189, UINT, 255, 1, false, 13,
   "ТИИ11", 190, UINT, 255, 1, false, 13,
   "ТИИ12", 191, UINT, 255, 1, false, 13,
   "ТИИ13", 192, UINT, 255, 1, false, 13,
   "ТИИ14", 193, UINT, 255, 1, false, 13,
   "ТИИ15", 194, UINT, 255, 1, false, 13,
   "Номинальный ток(А)", 200, REAL, 255, 1, false, 14,
   "Ток холостого хода(А)", 202, REAL, 255, 1, false, 14,
   "Номинальная скорость(об/мин)", 204, REAL, 255, 1, false, 14,
   "Номинальная частота(Гц)", 206, REAL, 255, 1, false, 14,
   "Номинальная мощность(Вт)", 208, REAL, 255, 1, false, 14,
   "КПД двигателя", 210, REAL, 255, 1, false, 14,
   "Min частота для установки Fmin(Гц)", 212, REAL, 255, 1, false, 14,
   "Max частота для установки Fmax(Гц)", 214, REAL, 255, 1, false, 14,
   "Характеристика U(f) №1: % напряжения от 380В в точке Fmin", 216, REAL, 255, 1, false, 15,
   "Характеристика U(f) №2: % напряжения от 380В в точке Fmin", 224, REAL, 255, 1, false, 15,
   "Характеристика U(f) №3: % напряжения от 380В в точке Fmin", 232, REAL, 255, 1, false, 15,
   "Характеристика U(f) №4: % напряжения от 380В в точке Fmin", 240, REAL, 255, 1, false, 15,
   "Характеристика U(f) №5: % напряжения от 380В в точке Fmin", 248, REAL, 255, 1, false, 15,
   "Характеристика U(f) №6: % напряжения от 380В в точке Fmin", 256, REAL, 255, 1, false, 15,
   "Характеристика U(f) №7: % напряжения от 380В в точке Fmin", 264, REAL, 255, 1, false, 15,
   "Характеристика U(f) №8: % напряжения от 380В в точке Fmin", 272, REAL, 255, 1, false, 15,
   "Характеристика U(f) №9: % напряжения от 380В в точке Fmin", 280, REAL, 255, 1, false, 15,
   "Характеристика U(f) №10: % напряжения от 380В в точке Fmin", 288, REAL, 255, 1, false, 15,
   "Характеристика U(f) №1: % напряжения от 380В в точке Fmid между Fmin и Fmax", 218, REAL, 255, 1, false, 16,
   "Характеристика U(f) №2: % напряжения от 380В в точке Fmid между Fmin и Fmax", 226, REAL, 255, 1, false, 16,
   "Характеристика U(f) №3: % напряжения от 380В в точке Fmid между Fmin и Fmax", 234, REAL, 255, 1, false, 16,
   "Характеристика U(f) №4: % напряжения от 380В в точке Fmid между Fmin и Fmax", 242, REAL, 255, 1, false, 16,
   "Характеристика U(f) №5: % напряжения от 380В в точке Fmid между Fmin и Fmax", 250, REAL, 255, 1, false, 16,
   "Характеристика U(f) №6: % напряжения от 380В в точке Fmid между Fmin и Fmax", 258, REAL, 255, 1, false, 16,
   "Характеристика U(f) №7: % напряжения от 380В в точке Fmid между Fmin и Fmax", 266, REAL, 255, 1, false, 16,
   "Характеристика U(f) №8: % напряжения от 380В в точке Fmid между Fmin и Fmax", 274, REAL, 255, 1, false, 16,
   "Характеристика U(f) №9: % напряжения от 380В в точке Fmid между Fmin и Fmax", 282, REAL, 255, 1, false, 16,
   "Характеристика U(f) №10: % напряжения от 380В в точке Fmid между Fmin и Fmax", 290, REAL, 255, 1, false, 16,
   "Характеристика U(f) №1: % напряжения от 380В в точке Fmax", 220, REAL, 255, 1, false, 17,
   "Характеристика U(f) №2: % напряжения от 380В в точке Fmax", 228, REAL, 255, 1, false, 17,
   "Характеристика U(f) №3: % напряжения от 380В в точке Fmax", 236, REAL, 255, 1, false, 17,
   "Характеристика U(f) №4: % напряжения от 380В в точке Fmax", 244, REAL, 255, 1, false, 17,
   "Характеристика U(f) №5: % напряжения от 380В в точке Fmax", 252, REAL, 255, 1, false, 17,
   "Характеристика U(f) №6: % напряжения от 380В в точке Fmax", 260, REAL, 255, 1, false, 17,
   "Характеристика U(f) №7: % напряжения от 380В в точке Fmax", 268, REAL, 255, 1, false, 17,
   "Характеристика U(f) №8: % напряжения от 380В в точке Fmax", 276, REAL, 255, 1, false, 17,
   "Характеристика U(f) №9: % напряжения от 380В в точке Fmax", 284, REAL, 255, 1, false, 17,
   "Характеристика U(f) №10: % напряжения от 380В в точке Fmax", 292, REAL, 255, 1, false, 17,
   "Характеристика U(f) №1: Fmid(Гц)", 222, REAL, 255, 1, false, 18,
   "Характеристика U(f) №2: Fmid(Гц)", 230, REAL, 255, 1, false, 18,
   "Характеристика U(f) №3: Fmid(Гц)", 238, REAL, 255, 1, false, 18,
   "Характеристика U(f) №4: Fmid(Гц)", 246, REAL, 255, 1, false, 18,
   "Характеристика U(f) №5: Fmid(Гц)", 254, REAL, 255, 1, false, 18,
   "Характеристика U(f) №6: Fmid(Гц)", 262, REAL, 255, 1, false, 18,
   "Характеристика U(f) №7: Fmid(Гц)", 270, REAL, 255, 1, false, 18,
   "Характеристика U(f) №8: Fmid(Гц)", 278, REAL, 255, 1, false, 18,
   "Характеристика U(f) №9: Fmid(Гц)", 286, REAL, 255, 1, false, 18,
   "Характеристика U(f) №10:Fmid(Гц)", 294, REAL, 255, 1, false, 18,
   "Количество полюсов", 296, UINT, 255, 1, false, 14,
   "Частота ШИМ (2-10 кГц)", 297, UINT, 255, 1, false, 14,
   "Режим отладочного порта:0 - непрерывный, 1 - ModBus Slave, 2 - Wi-Fi", 298, UINT, 255, 1, false, 8,
   "Контекст 1", 400, REAL, 255, 1, false, 19,
   "Контекст 2", 402, REAL, 255, 1, false, 19,
   "Контекст 3", 404, REAL, 255, 1, false, 19,
   "Контекст 4", 406, REAL, 255, 1, false, 19,
   "Контекст 5", 408, REAL, 255, 1, false, 19,
   "Контекст 6", 410, REAL, 255, 1, false, 19,
   "Контекст 7", 412, REAL, 255, 1, false, 19,
   "Контекст 8", 414, REAL, 255, 1, false, 19,
   "Контекст 9", 416, REAL, 255, 1, false, 19,
   "Контекст 10", 418, REAL, 255, 1, false, 19,
   "Контекст 1", 420, REAL, 255, 1, false, 20,
   "Контекст 2", 422, REAL, 255, 1, false, 20,
   "Контекст 3", 424, REAL, 255, 1, false, 20,
   "Контекст 4", 426, REAL, 255, 1, false, 20,
   "Контекст 5", 428, REAL, 255, 1, false, 20,
   "Контекст 6", 430, REAL, 255, 1, false, 20,
   "Контекст 7", 432, REAL, 255, 1, false, 20,
   "Контекст 8", 434, REAL, 255, 1, false, 20,
   "Контекст 9", 436, REAL, 255, 1, false, 20,
   "Контекст 10", 438, REAL, 255, 1, false, 20,
   "Контекст 1", 440, REAL, 255, 1, false, 21,
   "Контекст 2", 442, REAL, 255, 1, false, 21,
   "Контекст 3", 444, REAL, 255, 1, false, 21,
   "Контекст 4", 446, REAL, 255, 1, false, 21,
   "Контекст 5", 448, REAL, 255, 1, false, 21,
   "Контекст 6", 450, REAL, 255, 1, false, 21,
   "Контекст 7", 452, REAL, 255, 1, false, 21,
   "Контекст 8", 454, REAL, 255, 1, false, 21,
   "Контекст 9", 456, REAL, 255, 1, false, 21,
   "Контекст 10", 458, REAL, 255, 1, false, 21,
   "Контекст 1", 460, REAL, 255, 1, false, 22,
   "Контекст 2", 462, REAL, 255, 1, false, 22,
   "Контекст 3", 464, REAL, 255, 1, false, 22,
   "Контекст 4", 466, REAL, 255, 1, false, 22,
   "Контекст 5", 468, REAL, 255, 1, false, 22,
   "Контекст 6", 470, REAL, 255, 1, false, 22,
   "Контекст 7", 472, REAL, 255, 1, false, 22,
   "Контекст 8", 474, REAL, 255, 1, false, 22,
   "Контекст 9", 476, REAL, 255, 1, false, 22,
   "Контекст 10", 478, REAL, 255, 1, false, 22,
   "Контекст 1", 480, REAL, 255, 1, false, 23,
   "Контекст 2", 482, REAL, 255, 1, false, 23,
   "Контекст 3", 484, REAL, 255, 1, false, 23,
   "Контекст 4", 486, REAL, 255, 1, false, 23,
   "Контекст 5", 488, REAL, 255, 1, false, 23,
   "Контекст 6", 490, REAL, 255, 1, false, 23,
   "Контекст 7", 492, REAL, 255, 1, false, 23,
   "Контекст 8", 494, REAL, 255, 1, false, 23,
   "Контекст 9", 496, REAL, 255, 1, false, 23,
   "Контекст 10", 498, REAL, 255, 1, false, 23,
   "Контекст 1", 500, UINT, 255, 1, false, 24,
   "Контекст 2", 501, UINT, 255, 1, false, 24,
   "Контекст 3", 502, UINT, 255, 1, false, 24,
   "Контекст 4", 503, UINT, 255, 1, false, 24,
   "Контекст 5", 504, UINT, 255, 1, false, 24,
   "Контекст 6", 505, UINT, 255, 1, false, 24,
   "Контекст 7", 506, UINT, 255, 1, false, 24,
   "Контекст 8", 507, UINT, 255, 1, false, 24,
   "Контекст 9", 508, UINT, 255, 1, false, 24,
   "Контекст 10", 509, UINT, 255, 1, false, 24,
   "Контекст 1", 510, UINT, 255, 1, false, 25,
   "Контекст 2", 511, UINT, 255, 1, false, 25,
   "Контекст 3", 512, UINT, 255, 1, false, 25,
   "Контекст 4", 513, UINT, 255, 1, false, 25,
   "Контекст 5", 514, UINT, 255, 1, false, 25,
   "Контекст 6", 515, UINT, 255, 1, false, 25,
   "Контекст 7", 516, UINT, 255, 1, false, 25,
   "Контекст 8", 517, UINT, 255, 1, false, 25,
   "Контекст 9", 518, UINT, 255, 1, false, 25,
   "Контекст 10", 519, UINT, 255, 1, false, 25,
   "Источник контекста: 0-модбас, 1-ТС1", 520, UINT, 255, 1, false, 26,
   "Номер контекста при ТС1=0 (TC1 - источник контекста)", 521, UINT, 255, 1, false, 26,
   "Номер контекста при ТС1=1 (TC1 - источник контекста)", 522, UINT, 255, 1, false, 26,
   "Номер контекста, когда источник контекста - модбас", 523, UINT, 255, 1, false, 26,
   "Время перехода между контекстами(В/с)", 524, UINT, 255, 1, false, 26,
   "Насыщение ключей", 800, UINT, 255, 1, false, 27,
   "Перенапряжений", 803, UINT, 255, 1, false, 27,
   "Аварий мгновенных токов Ia", 804, UINT, 255, 1, false, 27,
   "Аварий мгновенных токов Ib", 805, UINT, 255, 1, false, 27,
   "Аварий мгновенных токов Ic", 806, UINT, 255, 1, false, 27,
   "Аварий действующего значения тока", 807, UINT, 255, 1, false, 27,
   "Аварий ЭТР", 808, UINT, 255, 1, false, 27,
   "Обрывов фазы A", 809, UINT, 255, 1, false, 27,
   "Обрывов фазы B", 810, UINT, 255, 1, false, 27,
   "Обрывов фазы C", 811, UINT, 255, 1, false, 27,
   "Дисбалансов тока", 812, UINT, 255, 1, false, 27,
   "Аварий температуры отсека конденсаторов", 813, UINT, 255, 1, false, 27,
   "Аварий температуры радиатора", 814, UINT, 255, 1, false, 27,
   "Низкого напряжения DC-шины", 815, UINT, 255, 1, false, 27,
   "Холостой ход и превышение частоты", 816, UINT, 255, 1, false, 27,
   "adc_pwm_err", 817, UINT, 255, 1, false, 27,
   "adc_energy_err", 818, UINT, 255, 1, false, 27,
   "Перезагрузок по Watchdog timer ", 830, UINT, 255, 1, false, 27,
   "Отвод №1", 900, UINT, 255, 1, false, 28,
   "Отвод №2", 901, UINT, 255, 1, false, 28,
   "Отвод №3", 902, UINT, 255, 1, false, 28,
   "Отвод №4", 903, UINT, 255, 1, false, 28,
   "Отвод №5", 904, UINT, 255, 1, false, 28,
   "Отвод №6", 905, UINT, 255, 1, false, 28,
   "Отвод №7", 906, UINT, 255, 1, false, 28,
   "Отвод №8", 907, UINT, 255, 1, false, 28,
   "Отвод №9", 908, UINT, 255, 1, false, 28,
   "Отвод №10", 909, UINT, 255, 1, false, 28,
   "Отвод №11", 910, UINT, 255, 1, false, 28,
   "Отвод №12", 911, UINT, 255, 1, false, 28,
   "Отвод №13", 912, UINT, 255, 1, false, 28,
   "Отвод №14", 913, UINT, 255, 1, false, 28,
   "Отвод №15", 914, UINT, 255, 1, false, 28,
   "Отвод №16", 915, UINT, 255, 1, false, 28,
   "Отвод №1", 916, UINT, 255, 1, false, 29,
   "Отвод №2", 917, UINT, 255, 1, false, 29,
   "Отвод №3", 918, UINT, 255, 1, false, 29,
   "Отвод №4", 919, UINT, 255, 1, false, 29,
   "Отвод №5", 920, UINT, 255, 1, false, 29,
   "Отвод №6", 921, UINT, 255, 1, false, 29,
   "Отвод №7", 922, UINT, 255, 1, false, 29,
   "Отвод №8", 923, UINT, 255, 1, false, 29,
   "Отвод №9", 924, UINT, 255, 1, false, 29,
   "Отвод №10", 925, UINT, 255, 1, false, 29,
   "Отвод №11", 926, UINT, 255, 1, false, 29,
   "Отвод №12", 927, UINT, 255, 1, false, 29,
   "Отвод №13", 928, UINT, 255, 1, false, 29,
   "Отвод №14", 929, UINT, 255, 1, false, 29,
   "Отвод №15", 930, UINT, 255, 1, false, 29,
   "Отвод №16", 931, UINT, 255, 1, false, 29,
   "Включение архивирования(>0)", 932, UINT, 255, 1, false, 30,
   "Включение формирования суммарного суточного замера по отводу(>0)", 933, UINT, 255, 1, false, 30,
   "Час формирования суммарного суточного замера по отводам", 934, UINT, 255, 1, false, 30,
   "Минута формирования суммарного суточного замера по отводам", 935, UINT, 255, 1, false, 30,
   "Секунда формирования суммарного суточного замера по отводам", 936, UINT, 255, 1, false, 30,
   "Смещение по времени формирования записей(+/-сек)", 937, INT, 255, 1, false, 30,
   "WATTH(кВт*час)", 890, REAL, 255, 1, false, 31,
   "Время UNIX (секунды с 01_01_2000г)", 16, UDINT, 255, 1, false, 32
};

const char AnswerOK[] = "HTTP/1.0 200 OK\r\n"
                          "Content-Type: text/plain; charset=windows-1251\r\n";

const char AnswerMain[] = "<html>\r\n"
                            "<frameset cols=\"300,*\">\r\n"
                            "<frame src=\"menu.htm\">\r\n"
                            "<frame src=\"0\" name=\"r\">\r\n"
                            "</frameset>\r\n"
                            "</html>\r\n";

const char AnswerError[] = 
        "HTTP/1.0 404 Not Found\r\n"
        "Content-Type: text/html; charset=windows-1251\r\n"
        "Server: ATmega16\r\n"
        "\r\n"
        "<pre>Page not found\r\n\r\n"
        "<a href='/'>Home page</a></pre>\r\n";

const char AnswerHead[] = "HTTP/1.0 200 OK\r\nContent-Length: ";

int PrevID;
char Flag_Wait_OK = 0;
char CountSecWaitOK;

char sAnswerID[15];
char sFindChanel[5];
char sDeleteItemOfChanel[10];
char sDoSendAT[50];
char sCreateMenuForBrowser[200];
char sCreatPageForBrowser[200];
char ValStr[20];
char sAddHead[50];
char head[50];
char sDoHTML[200];
char sID[10];

char strFloat1[10];
char strFloat2[10];
char debOut[100];
// прочитанные данные с устройства
uint16_t ValData[2];

TFloat val_Float;
TInteger val_Integer;
TDWord val_DWord;
//
TParamItem PItem;
TCharToParamItem CharToParamItem;
extern char CountWriteStack; //количество пакетов для записи;
extern char IndexWriteStack; // текущии индекс в стеке
char Flag_POST = 0;

// возвращает адрес следующего элемента, после копирования
unsigned char *copy_P(char *dest,const char *src)
//unsigned char *copy_P(unsigned char *dest,char __flash *src)
{
 //for (ii=0;ii<sizeof(webif_200_header)-1;ii++) *dest++=*src++; 
  while(*src) *dest++=*src++; 
  return(dest);
}

int CurrPos; // Текущаая позиция для функции Pos в CheskWordInString
unsigned int LengthInBuf;
// функция возвращает позицию первого вхождение подстроки substr в строке str начиная с позиции idx
// возвращает -1 если не найдено
int Pos(char *substr, char *str, int idx)
{
    int i, j, result = -1;
    enum bool flOK;
    int len_substr = strlen(substr); //длина подстроки без 0 символа
    int len_str = strlen(str); //длина строки
    
    for (i = idx; i <= len_str - len_substr; i++)
    {
        flOK = true;
        for (j = 0; j < len_substr; j++)
        {
            if (substr[j] != str[i + j])
            {
                flOK = false;
                break;
            }
        }
        if (flOK)
        {
            result = i;
            break;
        }
    }
    return result;
}

float ExtractFloat(char *aString, int aIndex, int aSize)
{
    int i, IntPart;
    long int koef;
    float result;
    
    koef = 1;
    result = 0;
    IntPart = 0;
    
    if (aIndex < 0)
    {
        for (i = aSize - 1; i >= 0; i--)
        {
            result = result + (aString[i] - 0x30) * koef;
            koef = koef * 10;
        }
        return result;      
    }
    
    for (i = aIndex - 1; i >=0; i--)
    {
         IntPart = IntPart + (aString[i] - 0x30) * koef;
         koef = koef * 10;
    };

    koef = 1;

    for (i = aSize - 1; i > aIndex; i--)
    {
         result = result + (aString[i] - 0x30) * koef;
         koef = koef * 10;
    };

    result = IntPart + result/koef;
    return result;
}

float StrToFloat(char *aString)
{
    int idx;
    int sz;
    float result;

    sz = strlen(aString);

    if (sz < 1)
        return 0;
    idx = Pos(",", aString, 0);
    if (idx < 0)
    {
        idx = Pos(".", aString, 0);
        if (idx < 0)
        {
            //целое значение
            result = ExtractFloat(aString, idx, sz);
        }
        else
        {
            //разделитель точка
            result = ExtractFloat(aString, idx, sz);
        }
    }
    else
    {
        //разделитель запятая
        result = ExtractFloat(aString, idx, sz);
    }
    return result;
}

void FloatToStr(float aValue, char *aString)
{
  int i, j;
  long int i1; 
  float f;
  unsigned long int i2;
  int sz;
  
  for (i = 0; i < 10; i++)
    strFloat2[i] = 0;
  
  i1 = aValue;
  f = aValue - i1;
  i2 = 1;
  for (i = 0; i < 6; i++)
  {
    i2 = f * 10;
    if (f * 10 - i2 == 0)
      break;
    f = f * 10;
  };
  
  sprintf(strFloat1,"%ld", i2);
  sz = strlen(strFloat1);
  sz = i - sz;
  if (sz > 0)
  {
    for (j = 0; j < sz; j++)
        strFloat2[j] = 0x30;
    sprintf(aString,"%ld.%s%ld", i1, strFloat2, i2);
  }
  else
  sprintf(aString,"%ld.%ld", i1, i2);
}

enum bool CheskWordInString(char *substr, char *str)
{
    int i;

    i = Pos(substr, str, CurrPos);
    
    if (i >= 0)
    {
        CurrPos = i;
        return  true;
    }
    else
        return false;
};

int StrToInt(char *str)
{
  int i, n, result, k, d;
  
  result = 0;
  k = 1;
  n = strlen(str);
  
  if (n < 1)
    return -1;
  
  for (i = n - 1; i >= 0; i--)
  {
      d = str[i] - 0x30;
      if ((d >= 0) && (d <= 9))
      {
         result = result + (str[i] - 0x30) * k;
         k = k * 10;
      }
      else
        return -1;
  }
  
  return result;
    
}

int GetAnswerID(char *str)
{
    int i, b, e, result;
    

    for (i = 0; i < 20; i++)
        sAnswerID[i] = 0;
    
    b = Pos("GET /", str, CurrPos) + 5;
    e = Pos(" HTTP/", str, CurrPos);
    
    if (b == e)
        return PAGE_MAIN;
    
    for (i = 0; i < e - b; i++)
        sAnswerID[i] = str[i + b];

    if (strcmp(sAnswerID, "menu.htm") == 0)
        return PAGE_MENU;

    if (strcmp(sAnswerID, "favicon.ico") == 0)
        return PAGE_ERROR;
    
    result = StrToInt(sAnswerID); //atoi(sAnswerID);
    
    if (( result > MAX_ARRSECTION - 1) || (result < 0))
        return PAGE_ERROR;
    
    PrevID = result;
    return result;
}

// Возвращает номер канала найденный после IPD+
int FindChanel(char *str)
{
    int i, j;
    enum bool flOK = true;
    
    for (i = 0; i < 5; i++)
        sFindChanel[i] = 0;
    
    i = Pos("+IPD,", str, CurrPos) + 5; // 4 - длина IPD+
    j = 0;
    
    while  (str[i] != ',')
    {
        sFindChanel[j] = str[i];
        j++;
        i++;
        if (i > strlen(str))
        {
            flOK = false;
            break;

        };
    };
    
    if (flOK)
        return  atoi(sFindChanel);
    else
        return -1;
};

void DeleteHtmlItem(int aIndex)
{
    int i;
    if (CountItemInHtmlList == 0)
        return;
    if (aIndex > MAX_HTML_ITEM - 1)
        return;
    for (i = aIndex; i <= MAX_HTML_ITEM - 2; i++)
        htmlList[i] = htmlList[i + 1];
    CountItemInHtmlList--;
}

void DeleteItemOfChanel(char * str)
{
    int i, j, chnl;
    

    if (CountItemInHtmlList == 0)
        return;

    j = Pos(",CLOSED", str, 0);

    //memcpy(sDeleteItemOfChanel, str + (j - 1), 1);
    //chnl = atoi(sDeleteItemOfChanel);
    chnl = str[j - 1] - 0x30;
    i = 0;
    while (true)
    {
        if (htmlList[i].Chanel == chnl)
            DeleteHtmlItem(i);
        else
            i++;

        if (i >= CountItemInHtmlList)
            break;
    };
};

void AddHtmlItemInList(ThtmlItem aItem)
{
    if (CountItemInHtmlList >= MAX_HTML_ITEM - 1)
        return;
    CountItemInHtmlList++;
    htmlList[CountItemInHtmlList - 1] = aItem;
};

void ExtractDataFromString(char *inBuf, unsigned int aSize)
{
    int i, idx1, idx2;
    
    CountWriteStack = 0; //количество пакетов для записи;
    IndexWriteStack = 0; // текущии индекс в стеке
    
    while (true)
    {
        idx1 = Pos(NAME_POST, inBuf, CurrPos);
        if (idx1 < 0)
            break;
        //Очищаем массивы для номера параметра и значения
        for (i = 0; i < 10; i++)
        {
            sID[i] = 0;
            ValStr[i] = 0;
        };
        CurrPos = idx1;
        idx1 = idx1 + strlen(NAME_POST); // позиция первой цифры в конструкции rinatXXX
        idx2 = Pos("=", inBuf, CurrPos);
        // Получаем номер параметра
        for(i = idx1; i < idx2; i++)
            sID[i - idx1] = inBuf[i];

        idx1 = idx2 + 1; // в idx1 позиция за символом '='
        idx2 = Pos("&", inBuf, CurrPos);

        if (idx2 < 0)
            idx2 = aSize;
        // Получаем значение параметра
        for(i = idx1; i < idx2; i++)
            ValStr[i - idx1] = inBuf[i];
        //преобразуем индекс в число
        i = atoi(sID);
        CurrPos = idx2;
        // Получаем сруктуру параметра по индексу
        PItem = GetParam(&ParamItem[i].Name[0]);
        WriteDataToMemory(ValStr);
        
        if (idx2 == aSize)
            break;
    };
};
// inBuf - входной\выходной буфер. *aSize - размер выходного пакета
char CreateHtmlItem(char *inBuf, unsigned int *aSize, unsigned int aSizeIn)
{
    int id;
    int chnl;
    ThtmlItem htmlItem;
    char result;
    
    result = NOT_WRITE_PORT;
     
    if (aSizeIn > 0)
    {
       
        CurrPos = 0;
        LengthInBuf = aSizeIn;
        
        if (CountItemInHtmlList > 0)
        {
          switch (htmlList[0].Step)
          { 
          case STEP_WAIT_SEND_OK:
              if (CheskWordInString("SEND OK", inBuf))
              {
                if (Flag_POST == 2)
                {
                  Flag_POST = 0;
                  if (CountWriteStack > 0)
                  {
                    comand_WiFi = 17;
                    rinatTimer = 0;
                  };
                };  
                DeleteHtmlItem(0);
                break;
              }
              else
                CountItemInHtmlList = 0;
              break;
           case STEP_WAIT_OK:
              if (CheskWordInString("OK\r\n>", inBuf))
              {
                Flag_Wait_OK = 0;
                if (Flag_POST == 1)
                  Flag_POST = 2;
                htmlList[0].Step = STEP_HTML;
	        DoHTML(htmlList[0].ID, inBuf, aSize);
                htmlList[0].Step = STEP_WAIT_SEND_OK;
                result = WRITE_PORT; 
                return result;
              }
              else
                CountItemInHtmlList = 0;
              break;
          };
        };
        
        //CurrPos = 0;
        //if (CheskWordInString("CLOSED", inBuf))
        //    DeleteItemOfChanel(inBuf);
        CurrPos = 0;


        while (true)
        {
            if (CheskWordInString("+IPD,", inBuf))
            {
                chnl = FindChanel(inBuf);

                if (CheskWordInString("GET /", inBuf))
                {
                    id = GetAnswerID(inBuf);

                    if (id >=0)
                    {
                        htmlItem.Chanel = chnl;
                        htmlItem.ID = id;
                        htmlItem.Step = STEP_CIPSEND;
                        AddHtmlItemInList(htmlItem);

                    };
                }
                else
                {

                    if (CheskWordInString("POST /", inBuf))
                    {
                        id = PrevID;

                        ExtractDataFromString(inBuf, aSizeIn); //Надо дописать
                        if (id >= 0)
                        {
                            Flag_POST = 1;
                            htmlItem.Chanel = chnl;
                            htmlItem.ID = id;
                            htmlItem.Step = STEP_CIPSEND;
                            AddHtmlItemInList(htmlItem);

                        };

                    };
                };
            }
            else
                break;

        };

        if (CountItemInHtmlList == 0)
            return result;

        switch (htmlList[0].Step)
        {
        case STEP_CIPSEND:
            DoSendAT(htmlList[0].Chanel, htmlList[0].ID, inBuf, aSize);
            htmlList[0].Step = STEP_WAIT_OK;
            Flag_Wait_OK = 1;
            CountSecWaitOK = 0;
            result = WRITE_PORT;
            break;
        case STEP_HTML:
            
	    DoHTML(htmlList[0].ID, inBuf, aSize);
            htmlList[0].Step = STEP_WAIT_SEND_OK;
            result = WRITE_PORT;
            break;
        };
        
    };
    return result;
};

void DoSendAT(int aID, int aAnswerID, char *inBuf, unsigned int *aSize)
{

    int i;
    for (i = 0; i < MAX_SIZE_HTML_PACKET; i++)
      HTMLString[i] = 0;
    
    switch (aAnswerID)
    {
    case PAGE_OK:
        copy_P(sDoHTML, AnswerOK);
        sprintf(sDoSendAT, "AT+CIPSEND=%d,%d\r\n", aID, strlen(sDoHTML));
        break;
    case PAGE_MAIN:
        copy_P(sDoHTML, AnswerMain);
        CreateHead(sDoHTML);
        sprintf(sDoSendAT, "AT+CIPSEND=%d,%d\r\n", aID, strlen(head) + strlen(sDoHTML));
        break;
    case PAGE_MENU:
        CreateMenuForBrowser();
        CreateHead(HTMLString);
        sprintf(sDoSendAT, "AT+CIPSEND=%d,%d\r\n", aID, strlen(head) + strlen(HTMLString));
        break;
    case PAGE_ERROR:
        copy_P(sDoHTML, AnswerError);
        sprintf(sDoSendAT, "AT+CIPSEND=%d,%d\r\n", aID, strlen(sDoHTML));
        break;
    default:
        CreatPageForBrowser(aAnswerID);
        CreateHead(HTMLString);
        sprintf(sDoSendAT, "AT+CIPSEND=%d,%d\r\n", aID, strlen(head) + strlen(HTMLString));
        
    };
    
    
    sprintf(inBuf,"%s", sDoSendAT);
    *aSize = strlen(inBuf);
    
};

// Меню для браузера
void CreateMenuForBrowser()
{
    int i, j;
    
    sprintf (HTMLString , "%s", "<html>\r\n <body link=\"white\" alink =\"yellow\" vlink=\"white\" style=\"background: #0066FF\">\r\n");
                                          
    for (i = 0; i <= MAX_ARRSECTION - 1; i++)
    {
        for (j = 0; j < SECTION_SIZE; j++)
            SectionName[j] = 0;
        
        copy_P(SectionName, ArrSection[i]);
        sprintf(sCreateMenuForBrowser, "<li><a href =\"%d\" target =\"r\">%s</a></li>\r\n", i, SectionName);
        //sprintf(sCreateMenuForBrowser, "<li><a href =\"%d\" target =\"r\" style=\"color: white; text-decoration: none\">%s</a></li>\r\n", i, SectionName);
        strcat(HTMLString, sCreateMenuForBrowser);

    };
    strcat(HTMLString, "</body>\r\n </html>\r\n");
        
};
TParamItem GetParam(const char *src)
{
    int i;


    for (i = 0; i < sizeof(TParamItem); i++)
        CharToParamItem.buf[i] = *src++;

    return CharToParamItem.Pitem;
};

char GetCountWord(void)
{
    switch (PItem.TypeValue)
    {
    case INT: return 1;
    case UINT: return 1;
    case DINT: return 2;
    case UDINT: return 2;
    case REAL: return 2;
    default: return 0;
    };
};

unsigned int GetMaskInt(char aBitFieldStart, char aBitFieldCount)
{
    int i;
    unsigned int result, d;

    result = 0;
    for (i = aBitFieldStart; i < aBitFieldStart + aBitFieldCount; i++)
    {
        d = 1 << i;
        result = result + d;
    };
    return result;
};

unsigned long int GetMaskWord(char aBitFieldStart, char aBitFieldCount)
{
    int i;
    unsigned long int result, d;
    result = 0;
    for (i = aBitFieldStart; i < aBitFieldStart + aBitFieldCount; i++)
    {
        d = 1 << i;
        result = result + d;
    };
    return result;
};

char WriteDataToMemory(char *aValStr)
{
    char sz;
    int val_int;
    unsigned long int mask32, val_long;
    unsigned int mask16;
    
    
    // sz -количество слов которое надо прочитать, если 0 то ничего не делать
    sz = GetCountWord();
    if (sz == 0)
        return 0;

    if (read_modbus(READ_DATA, PItem.Adres, sz, ValData) == 0)
        return 0;
    
       
    switch (PItem.TypeValue)
    {
    case INT: 
        val_int = atoi(aValStr);
        if (PItem.isBitField)
        {
            mask16 = GetMaskInt(PItem.BitFieldStart, PItem.BitFieldCount);
            ValData[0] = ValData[0] & (~mask16);
            val_int = val_int << PItem.BitFieldStart;
            ValData[0] = ValData[0] | val_int;

        }
        else
            ValData[0] = val_int;

        break;
    case UINT: 
        val_int = atoi(aValStr);
        if (PItem.isBitField)
        {
            mask16 = GetMaskInt(PItem.BitFieldStart, PItem.BitFieldCount);
            ValData[0] = ValData[0] & (~mask16);
            val_int = val_int << PItem.BitFieldStart;
            ValData[0] = ValData[0] | val_int;

        }
        else
            ValData[0] = val_int;
        
        break;
    case DINT: 

        val_long = atol(aValStr);
        if (PItem.isBitField)
        {
            val_Integer.Word[0] = ValData[0];
            val_Integer.Word[1] = ValData[1];          
            mask32 = GetMaskInt(PItem.BitFieldStart, PItem.BitFieldCount);
            val_Integer.Value = val_Integer.Value & (~mask32);
            val_long = val_long << PItem.BitFieldStart;
            val_Integer.Value = val_DWord.Value | val_long;
            ValData[0] = val_Integer.Word[0];
            ValData[1] = val_Integer.Word[1];
        }
        else
        {
            val_Integer.Value = val_long;
            ValData[0] = val_Integer.Word[0];
            ValData[1] = val_Integer.Word[1];

        };
        break;
    case UDINT:
        
        val_long = atol(aValStr);
        if (PItem.isBitField)
        {
            val_DWord.Word[0] = ValData[0];
            val_DWord.Word[1] = ValData[1];
            mask32 = GetMaskInt(PItem.BitFieldStart, PItem.BitFieldCount);
            val_DWord.Value = val_Integer.Value & (~mask32);
            val_long = val_long << PItem.BitFieldStart;
            val_DWord.Value = val_DWord.Value | val_long;
            ValData[0] = val_DWord.Word[0];
            ValData[1] = val_DWord.Word[1];
        }
        else
        {
            val_DWord.Value = val_long;
            ValData[0] = val_DWord.Word[0];
            ValData[1] = val_DWord.Word[1];

        };
               
        break;
    case REAL:
        val_Float.Value = atof(aValStr);//StrToFloat(aValStr);
        ValData[0] = val_Float.Word[0];
        ValData[1] = val_Float.Word[1];
        break;

  default: return 0;
  };
        
        
    if (read_modbus(WRITE_DATA, PItem.Adres, sz, ValData) == 0)
        return 0;
    return 1;
};

char ReadDataFromMemory(char *aValStr)
{
    char sz;
    int val_int;
    unsigned long int mask32;
    unsigned int mask16;
    
    // sz -количество слов которое надо прочитать, если 0 то ничего не делать
    sz = GetCountWord();
    if (sz == 0)
        return 0;

    if (read_modbus(READ_DATA, PItem.Adres, sz, ValData) == 0)
        return 0;
    
    switch (PItem.TypeValue)
    {
    case INT: 
        val_int = (int)ValData[0];
        if (PItem.isBitField)
        {
            mask16 = GetMaskInt(PItem.BitFieldStart, PItem.BitFieldCount);
            val_int = val_int & mask16;
            val_int = val_int >> PItem.BitFieldStart;
            ValData[0] = val_int;

        };
        sprintf(aValStr, "%d", val_int);
        break;
    case UINT: 
        if (PItem.isBitField)
        {
            mask16 = GetMaskInt(PItem.BitFieldStart, PItem.BitFieldCount);
            ValData[0] = ValData[0] & mask16;
            ValData[0] = ValData[0] >> PItem.BitFieldStart;

        };
        sprintf(aValStr, "%d", ValData[0]);
        break;
    case DINT: 
        val_Integer.Word[0] = ValData[0];
        val_Integer.Word[1] = ValData[1];
        if (PItem.isBitField)
        {
            mask32 = GetMaskInt(PItem.BitFieldStart, PItem.BitFieldCount);
            val_Integer.Value = val_Integer.Value & mask32;
            val_Integer.Value = val_Integer.Value >> PItem.BitFieldStart;
        };
        sprintf(aValStr, "%ld", val_Integer.Value);
        break;
    case UDINT:
        val_DWord.Word[0] = ValData[0];
        val_DWord.Word[1] = ValData[1];
        if (PItem.isBitField)
        {
            mask32 = GetMaskInt(PItem.BitFieldStart, PItem.BitFieldCount);
            val_DWord.Value = val_DWord.Value & mask32;
            val_DWord.Value = val_DWord.Value >> PItem.BitFieldStart;
        };
        sprintf(aValStr, "%ld", val_DWord.Value);
        break;
    case REAL:
        val_Float.Word[0] = ValData[0];
        val_Float.Word[1] = ValData[1];
        //FloatToStr(val_Float.Value, aValStr);
        sprintf(aValStr, "%f", (double)val_Float.Value);
        break;

  default: return 0;
  };
  
    return 1;
};

// страница с параметрами для браузера
void CreatPageForBrowser(int aIndex)
{
    int i;
    sprintf (HTMLString , "%s", "<html>\r\n <body>\r\n");           
    sprintf (sCreatPageForBrowser , "%s","<form action='/' method='POST'>\r\n");
    strcat(HTMLString, sCreatPageForBrowser);
    strcat(HTMLString, "<br>\r\n");
    
    for (i = 0; i < SECTION_SIZE; i++)
        SectionName[i] = 0;
    copy_P(SectionName, ArrSection[aIndex]);

    sprintf (sCreatPageForBrowser , "<font color=\"#ff0000\">Раздел: %s</font>\r\n", SectionName);
    strcat(HTMLString, sCreatPageForBrowser);
    strcat(HTMLString, "<br>\r\n");
    
    for (i = 0; i <= MAX_PARAM_ITEM - 1; i++)
    {
        
        PItem = GetParam(&ParamItem[i].Name[0]);
                
        if (PItem.SectionIndex == aIndex)
        {
            ReadDataFromMemory(ValStr);
            sprintf(sCreatPageForBrowser, "%s\r\n", PItem.Name);
            strcat(HTMLString, sCreatPageForBrowser);
            sprintf(sCreatPageForBrowser, "<input type='text' name='%s%d' size='10' value='%s'>\r\n", NAME_POST, i, ValStr);
            strcat(HTMLString, sCreatPageForBrowser);
            strcat(HTMLString, "<br>\r\n");
        };
    };
    strcat(HTMLString, "<input type='submit' value='OK'>\r\n</form>\r\n");
    strcat(HTMLString, "</body>\r\n </html>\r\n");
};
void CreateHead(char *aString)
{
    // Создаем заголовок с длиной пакета
    copy_P(sAddHead, AnswerHead);
    sprintf(head, "%s%d\r\n\r\n", sAddHead, strlen(aString));
    
};
// Добавляет заголовок
void AddHead(char *aString, char a)
{
    int i, n;
    
    // Создаем заголовок с длиной пакета
    copy_P(sAddHead, AnswerHead);
    sprintf(head, "%s%d\r\n\r\n", sAddHead, strlen(aString));
    
    // если а = 0 то aString не HTMLString и можно просто присоеденить
    if (a == 0)
    {
        sprintf(HTMLString, "%s%s\r\n", head, aString);
        return;
    };
    //Если aString это HTMLString           
    //длина заголовка вместе с длиной пакет
    n = strlen(head);
    
    ShiftToRight(aString, n);
    
    for (i = 0; i < n; i++ )
        aString[i] = head[i];
    
};

void ShiftToRight(char *aString, int aCount)
{
    int i;
    
    for (i = strlen(aString) - 1; i >= 0; i--)
        aString[i + aCount] = aString[i];

};

void DoHTML(int aAnswerID, char *inBuf, unsigned int *aSize)
{
    int i;
    for (i = 0; i < MAX_SIZE_HTML_PACKET; i++)
      inBuf[i] = 0;
    
    switch (aAnswerID)
    {
    case PAGE_OK:
        sprintf (inBuf , "%s", sDoHTML);
        break;
    case PAGE_MAIN:
        sprintf(inBuf, "%s%s", head, sDoHTML);
        break;
    case PAGE_MENU:
        sprintf(inBuf, "%s%s", head, HTMLString);
        break;
    case PAGE_ERROR:
        sprintf (inBuf , "%s", sDoHTML);
        break;
    default:
        sprintf(inBuf, "%s%s", head, HTMLString);
        break;
    };
    
    *aSize = strlen(inBuf);

};

