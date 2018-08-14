﻿/* --- NOTES: --- */
// SRV всегда отвечает только на последнюю команду (т.е. буффера/аккумулятора команд в SRV нет), т.е. если послать подряд 2 команды записи, то если первая команда не выполнилась до получения второй, 
// то она и не выполнится автоматически после выполнения второй команды, т.е. нужно будет первую команду посылать заново после выполнения второй команды.
// Таким образом необходимо после отправки любой из команд дождаться ответа от SRV о её выполнении и только потом отправлять следующую команду.

// При отправке команды записи/установки значения параметра/режима/контрольного слова и т.п. SRV в ответ после удачного выполнения команды присылает команду 0х60 CMD_LOW CMD_HIGH 0x00 0x00 0x00 0x00 0x00,
// т.е. например, при записи ускорения 0x23 0x83 0x60 0x00 0x00 0xF0 0x00 0x00 (CMD = 0x6083, CMD_HIGH = 0x83, CMD_LOW = 0x860) SRV при удачном выполнении пришлет 0x60 0x83 0x60 0x00 0x00 0x00 0x00 0x00,
// т.е. в ответе от SRV реальное записанное значение параметра не получаем.
// Поэтому для контроля верности записи нужно перечитать реально записавшееся в SRV значение параметра после получения подтверждения от SRV об удачном выполнении записи в ответ на нашу команду записи (команда 0х40 CMD_LOW CMD_HIGH 0x00 0x00 0x00 0x00 0x00).

// Например, для записи параметра ускорения порядок сообщений будет следующим:
// 0x23 0x83 0x60 0x00 0x00 0xF0 0x00 0x00 - мы отправляем команду записи
// 0x60 0x83 0x60 0x00 0x00 0x00 0x00 0x00 - SRV отвечает, что запись прошла успешно
// 0x40 0x83 0x60 0x00 0x00 0x00 0x00 0x00 - мы считываем реально записавшееся в SRV значение ускорения
// 0x43 0x83 0x60 0x00 0x00 0xF0 0x00 0x00 - SRV присылает нам значение ускорения, которое записано в него в данный момент

// Если заданное в 0x23 и считанное в 0x43 значения совпадают, то запись прошла успешно, команда считается выполненной, и нам можно отсылать новую команду к SRV для её выполнения.

// В случае ошибки при выполнении команды SRV высылает сообщение вида 0х80 CMD_LOW CMD_HIGH 0x00 0x00 0x00 0x00 0x00 (TODO: высылается ли причина ошибки - уточнить в описании протокола CANOpen)

// Формат команды: TYPEMSG CMD_LOW CMD_HIGH 0x00 VAL_LOWEST VAL_LOW VAL_HIGH VAL HIGHEST, т.е. если хотим записать ускорение (параметр 0x6023) = 439041102 единиц (439041102(dec) = 0х1A2B3C4E(hex)), то отправляем
// 0x23 0x83 0x60 0x00 0x4E 0x3C 0x2B 0x1A

// а если хотим записать ускорение = 1, то отправляем
// 0x23 0x83 0x60 0x00 0x01 0x00 0x00 0x00 

/*---*/
// Настраиваем TPDO1 для выдачи статуса(состояния) SRV по событию его изменения.
// 1. Вначале нужно сбросить все настройки TPDO1:                                 0x2F 0x00 0x1A 0x00 0x00 0x00 0x00 0x00
// 2. Далее нужно для TPDO1 назначить выдачу необходимого нам значения параметра: 0x23 0x00 0x1A 0x01 0x10 0x00 0x41 0x60 (1A00.1 = 0x6041 - статус SRV, в ответе каждый бит имеет значение, нас интересует 10й бит - Target reached. Target reached = 1 - это состояние SRV, при котором можно отправлять команду движения к SRV)
// 3. Далее нужно разрешить выдачу TPDO1 в параметре 1800.1:                      0x23 0x00 0x18 0x01 0x83 0x01 0x00 0x40 (1800.1 = 0x40000183)
// 4. Далее нужно назначить условие выдачи TPDO1 в параметре 1800.2:              0x2F 0x00 0x18 0x02 0xFF 0x00 0x00 0x00 (1800.2 = 0xFF) (0xFF - выдача TPDO1 по изменению значения назначенного в TPDO параметра 
//                                                                                                                                        (т.е. по СОБЫТИЮ, если в 1800.5(TPDO1 timer) записан 0х00 и по ТАЙМЕРУ, если в 1800.5 записано отличное от 0x00 значение (в миллисекундах)))
// 5. Далее в параметре 1A00.0 нужно указать, сколько TPDO-сообщений будут активны (будут отсылаться нам от SRV):
//                                                                                0x2F 0x00 0x1A 0x00 0x01 0x00 0x00 0x00 (1A00 = 0x01 - активируем первое TPDO1-сообщение, в каждую группу TPDOx можно назначить до 8ми сообщений)
// 6. Далее нужно подать команду Start Remote, которая запускает механизм выдачи TPDO от SRV к нам:
//                                                           0x00 0x00 0x00 0x00  0x01 0x03 0x00 0x00 0x00 0x00 0x00 0x00 , где 0x03 - SRV_CAN_Id/КАН-адрес привода. Идентификаторы получателя для MNT-сообщений равны нулю (т.е. CAN_Id сообщения(!!)/Msg_CAN_Id = 0x00 0x00 0x00 0x00)

/*---*/
// Для сообщений записи-чтения Msg_CAN_Id следующие:
// 1. Если сообщение от нас к SRV, то Msg_CAN_Id = 0x00 0x00 0x06 0x03, где 0x03 - SRV_CAN_Id (КАН-адрес привода), т.е Msg_CAN_Id = 0x600 + SRV_CAN_Id = 0x603 в данном случае
// 2. Если сообщение от SRV к нам, то Msg_CAN_Id = 0x00 0x00 0x05 0x83, где 0x03 (в байте 0x83) - SRV_CAN_Id (КАН-адрес привода), т.е Msg_CAN_Id = 0x580 + SRV_CAN_Id = 0x583 в данном случае
// 3. TPDO-сообщения имеют Msg_CAN_Id вида 0x183 для TPDO1 например (т.е. Msg_CAN_Id = 0x0n80 + SRV_CAN_Id, где n - порядковый номер TPDO-группы, например, n = 1 для TPDO1, n = 2 для TPDO2 и т.д.)

/*---*/
// !!! Значение Р1-55 (Max Profile Speed) ограничивает скорость мотора во всех режимах, в CANOpen пишется/читается через 0x607F.
// !!! Для Режима управления моментом этот параметр ограничивает скорость вращения двигателя. Актуально для больших моментов, иначе двигатель раскручивается до максимальной своей скорости.
// !!! При любом значении 0x607F значение Profile Velocity (0x6081) при чтении будет выдавать записанное значение, даже если оно будет выше, чем 0x607F, но фактически скорость мотора будет в случае (0x607F < 0x6081) ограничена значением 0x607F.



/* Если хотим интервалы, когда LED = on и когда LED = off разной длины, то нужно сделать так:

if(passedTimeout(tmr, 300))
{
LED_on();
tmr = setTimeout();
  }
    else if(passedTimeout(tmr, 100))
{
LED_off();
  } */


/* вывод в консоль отладки IAR в hex-виде */
// printf("SRV_current_params.tpdo_val: %04X \n", SRV_current_params.tpdo_val);
// printf("SRV_Target_Reached_bit:      %04X \n", SRV_Target_Reached_bit);  


/* --- TODO: --- */
// сделать команду записи и чтения значения Htbt_time через SUN_CAN команды и интерфейс плагина Control_Center