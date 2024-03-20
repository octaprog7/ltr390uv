Модуль MicroPython для управления LTR390UV. Датчик внешней освещенности в видимом и ультрафиолетовом диапазонах.
MicroPython module for controlling the LTR390UV. Ambient light sensor in visible and ultraviolet ranges.

# Описание
LTR-390UV-01 — это низковольтный датчик внешней освещенности (ALS) и датчик ультрафиолетового света (UV) в одном корпусе размером 2x2 мм.
Он обеспечивает линейный выход в широком динамическом диапазоне и хорошо подходит для применений в условиях высокой освещенности окружающей среды.

# Применения
Определение УФ-индекса окружающего света помогает людям эффективно защитить себя от солнечных ожогов, рака или повреждение глаз.
Для управления яркостью и цветом панели дисплея в мобильных, компьютерных и потребительских устройствах.

# Питание
Напряжение питания LTR390UV 3,3 В (от 1,71 В до 3,6 В)!

## Адрес датчика
После сканирования шины I2C, датчик обнаружился по адресу 0x53.

# Шина I2C
Просто подключите контакты (VCC, GND, SDA, SCL) платы с LTR390UV к соответствующим контактам Arduino, 
ESP или любой другой платы с прошивкой MicroPython! Подайте питание на плату.

# Загрузка ПО в плату
Загрузите прошивку micropython на плату NANO(ESP и т. д.), а затем файлы: main.py, ltr380uv.py и папку sensor_pack_2 полностью!
Затем откройте main.py в своей IDE и запустите/выполните его.

# Режимы работы датчика
## Режим ALS. Измерение освещенности в видимом человеком диапазоне, 400-800 нанометров.
Доступны значения в люксах и в 'сыром'-raw виде. Формула ALS Lux вызывается в методе get_illumination.

## Режим UV. Измерение освещенности в ультрафиолетовом диапазоне, 280-400 нанометров.
Доступны значения только в 'сыром'-raw виде. UVI формула преобразования мне не понятна, поэтому я ее не реализовал! 
Если у вас есть предложения, пишите!

# метод soft_reset
Метод вызывает програмный сброс датчика, но он не просыпается после него. Пока не вызывайте этот метод!

# Разрешение отсчета освещенности в битах
Разрешение рассчитывается автоматически по значению поля meas_rate метода start_measurement. Значение 13 бит не используется!

| meas_rate (raw) | meas_rate ms | resolution (raw) | resolution bit in sample | conversion time ms |
|-----------------|--------------|------------------|--------------------------|--------------------|
| 0               | 25           | 4                | 16                       | 25                 |
| 1               | 50           | 3                | 17                       | 50                 |
| 2               | 100          | 2                | 18                       | 100                |
| 3               | 200          | 1                | 19                       | 200                | 
| 4               | 500          | 0                | 20                       | 400                | 
| 5               | 1000         | 0                | 20                       | 400                | 

Во всех методах, входные параметры задаются в 'сыром'-raw виде!

# Параметр gain
Что такое 'Gain Range' спрашивайте у разработчиков датчика. Я не в курсе!

| gain (raw) | Gain Range |
|------------|------------|
| 0          | 1          |
| 1          | 3          |
| 2          | 6          |
| 3          | 9          |
| 4          | 18         |

# Ультрафиолет
Если вы собираетесь измерять ультрафиолетовое излучение этим датчиком, знайте, что УФ-излучение разрушает многие материалы.
Поэтому, датчик должен быть защищен от его воздействия большую часть времени, например, с помощью тонированного стекла. 
Для измерения ультрафиолета, защита(тонированное стекло) механически должна сдвигаться на несколько секунд. За это время можно произвести несколько измерений.
После этого нужно вернуть защиту на датчик! Иначе, через некоторое время, плата, на которой находится датчик перестанет работать.

# Плата с датчиком LTR390UV
![alt text](https://github.com/octaprog7/ltr390uv/blob/master/pics/board_ltr390.jpg)
# Среда разработки (IDE)
## ALS mode
![alt text](https://github.com/octaprog7/ltr390uv/blob/master/pics/lux_mini.png)
## uv_mode (IDE)
![alt text](https://github.com/octaprog7/ltr390uv/blob/master/pics/uv_mini.png)
## Внешний Источник Ультрафиолета
![alt text](https://github.com/octaprog7/ltr390uv/blob/master/pics/uv_convoy.jpg)
