
//Код
#define ST_PIT_ON (1) //Контроллер:
// подкод 
#define RESET_POWER (1) //Включение питания
#define RESET_EXT   (2) //Внешний сброс
#define RESET_BROWN (3) //Бросок по питанию
#define RESET_WDR   (4) //Сработка сторожевого таймера


//Код
#define ST_ERROR (2)   //Ошибка контролллера:
// подкод 
#define ERR1   (1) //Ошибка №1
#define ERR2   (2) //Ошибка №2
#define ERR3   (3) //Ошибка №3
#define ERR4   (4) //Ошибка №4
#define ERR5   (5) //Ошибка №5
#define ERR6   (6) //Ошибка №6
#define ERR7   (7) //Ошибка №7
#define ERR8   (8) //Ошибка №8
#define ERR9   (9) //Ошибка №9
#define ERR10   (10) //Ошибка №10
#define ERR11   (11) //Ошибка №11

//Код
#define ST_MODEM (3)   //Модем:
// подкод 
#define MDM_ON      (1) //Включение
#define MDM_SPEED   (2) //Скорость установлена
#define MDM_INIT    (3) //Инициализация выполнена
#define MDM_CON     (4) //Связь установлена
#define MDM_OFF     (5) //Выключение
#define BASE_SIM    (6) //основная симка   //dobavka
#define RES_SIM     (7) //резервная симка  //dobavka
#define MDM_KOFF    (8) //Выключение

//Код
#define ST_SET_PPP  (4)   //Установка соединения:
// подкод 
#define SET_PPP_TM   (1) //превышено время установления
#define SET_PPP_OK   (2) //Соединение установлено
#define SET_PPP_RJ   (3) //Отклонение соединения
#define SET_PPP_DCD  (4) //Потеря несущей
#define SET_PPP_CTS  (5) //Отсутствие CTS


//Код
#define ST_PPP  (5)   //Состояние "соединение":
// подкод 
#define PPP_RJ   (1) //Отклонение соединения
#define PPP_DCD  (2) //Потеря несущей
#define PPP_CTS  (3) //Отсутствие CTS
#define PPP_LS   (4) //Отсутствие связи с "сервером связи"
#define PPP_CH_SIM  (5) //Переключение на резервную SIM

//Код
#define ST_CLR_PPP  (6)   //Разрыв соединения:
// подкод 
#define CLR_PPP_RDY  (1) //готовность модема к разрыву
#define CLR_PPP_OK   (2) //разрыв выполнен корректно
#define CLR_PPP_ERR  (3) //разрыв выполнен с ошибкой

//Код
#define ST_DEBUG  (32)   //Отладка
// подкод 
#define POINT1  (1) //
#define POINT2  (2) //
#define POINT3  (3) //
