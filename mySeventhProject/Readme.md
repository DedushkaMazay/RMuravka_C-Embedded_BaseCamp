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

Action options: //Варианты действий

        1 - Clear the memory            //Очистить всю память
        2 - Write to memory             //Запись капсулы времени
        3 - Read the contents of memory //Прочитать содержимое памяти
        4 - Clear memory sector         //Очистить 1 сектор (4Кбайта)
        

Your choice(After entering, press ENTER): //N, F, 1-3.


При вводе нужно подтвердить запись нажатием кнопки Enter(при условии если она определена знаком \r)

Также можно редактировать ввод нажатием кнопки Backspace (при условии что она определена символом \b)

При вводе неправильных значений вы будете уведомлены и возращенны в главное Меню


Вы также можете по желанию изменнить временную капсулу(Макс. кол-во символов в строке: 4096 && Макс. кол-во строк: 511)


!!!УДАЧИ!!!


