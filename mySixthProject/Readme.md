Это программа полностью контролируется через UART
UART:
Скорость: 115200
Кол-во битов: 8
Стоп-бит: 1
Четность: Нет
Управление потоком: Нет
Функциональные клавиши: Linux
Клавиша Backspace: Control-H (\b)
Клавиша Enter: \r

Меню управления состоит из:
!!!PWM ON BUT NO LED SELECTED!!! //Состояние ШИМ
        N - Turn on the PWM; //Включение
        1 - LED selection;(Currently: no LED is on) //Выбор светодиода с текущим состоянием
        2 - Change the PWM frequency;(Currently: 0025 Hz)//Выбор частоты с текущим состоянием
        3 - Change Duty Cycle %;(Currently: 0005 %)//Выбор Скважности с текущим состоянием
        F - Turn off the PWM; //Выключение

Your choice(After entering, press ENTER): //N, F, 1-3.

При вводе нужно подтвердить запись нажатием кнопки Enter(при условии если она определена знаком \r)
Также можно редактировать ввод нажатием кнопки Backspace (при условии что она определена символом \b)
При вводе неправильных значений вы будете уведомлены и возращенны в главное Меню


!!!УДАЧИ!!!
