/* mbed Microcontroller Library
 * Copyright (c) 2018 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"

#include "CANopen.h"
#include "config.h"
#include <cstdint>
#include <string>

extern "C" {
#include "CO_driver.h"
}
#include "CO_Indicators.h"

// модуль с определениями констант EPOS2
#include "epos.h"

// флаг для остановки потоков
#define STOP_THREADS_FLAG   1
// флаг, разрешающий ввод параметров
#define ENABLE_SERIAL       1
// интервал работы потока в микросекундках, всегда должен быть равен одной мс
static const int RTTHREAD_INTERVAL_1000US = 1000;
// переменная увеличивается каждую милисекунду
volatile uint16_t CO_timer1ms = 0U;
// объект потока, в котором будет работать петля реального времени
Thread* rtThread;
// функция, исполняемая в потоке реального времени
static void rtTask(void);
// таймер для измерения производительности петли реального времени
Timer rtPerfTimer;
// функция для чтения параметров из последовательного порта
char * read_from_serial();


DigitalOut userLed(LED2, 1);
DigitalIn userButton(USER_BUTTON);

// последовательный порт
Serial      pc(USBTX, USBRX);
const int kMaxBufferSize = 100;
char      buffer[kMaxBufferSize];




// main() работает в собственном потоке в ОС
int main() 
{
    pc.baud(57600);
    printf("######## CANOPEN DEMO (" __DATE__ ", " __TIME__ ") ##########\n");
    OD_powerOnCounter++;    // инкрементировать число запусков

    CO_NMT_reset_cmd_t resetCmd = CO_RESET_NOT;
    while(resetCmd != CO_RESET_APP && resetCmd != CO_RESET_QUIT)
    {
        
        // параметры цикла
        uint8_t repeats = 3; // количество повторений цикла

        int32_t alpha = 200000; // позиция alpha в position units
        int32_t beta = -200000; // позиция beta в position units
        uint32_t profile_velocity = 1000; // скорость движения
        uint32_t profile_acceleration = 100; // ускорение
        uint32_t profile_deceleration = 100; // торможение

        // попробовать прочесть данные из последовательного порта. Может быть отключено при отладке
        if(ENABLE_SERIAL != 0){
            printf("Number of repeats: ");
            repeats = atoi(read_from_serial()); 
            printf("%i\n", repeats);

            printf("Alpha position: ");
            alpha = atoll(read_from_serial()); 
            printf("%i\n", alpha);
            printf("Beta position: ");
            beta = atoll(read_from_serial()); 
            printf("%i\n", beta);
            printf("Profile velocity: ");
            profile_velocity = atoll(read_from_serial()); 
            printf("%i\n", profile_velocity);
            printf("Profile acceleration: ");
            profile_acceleration = atoll(read_from_serial()); 
            printf("%i\n", profile_acceleration);
            printf("Profile deceleration: ");
            profile_deceleration = atoll(read_from_serial());
            printf("%i\n", profile_deceleration);
        }

        int8_t mode = 1; // режим работы, соответствует PPM
        uint16_t controlword = SHUTDOWN; // слово управления. SHUTDOWN для проведения перезагрузки в начале цикла
        int16_t motion_profile_type = 0;

        bool cw = false;  // направление движения. True - по часовой. Первое движение по часовой будет произведено в цикле инициализации
        bool home = false; // флаг, определяющий, что была отдана команда на движение к стартовой позиции
        bool configured = false; // флаг, определяющий, что была проведена конфигурация устройств

        int32_t start = 0; // начальная позиция

        CO_SDOclient_return_t res; // результат передачи SDO

        
        
        
        printf("####### start comms-loop #######\n");

        // инициализировать CANopen
        CO_ReturnError_t err = CO_init((void*)CAN_MODULE_ADDRESS, CANOPEN_DEFAULT_NODE_ID, CAN_BITRATE);
        if(err != CO_ERROR_NO){
            printf("ERROR: CO_init, code: %d\n", err);
            ThisThread::sleep_for(3000);   // в мс
            break;
        }

        // запустить в отдельном потоке обработку RPDO, TPDO и SYNC
        printf("Enabling rtThread processing\n");
        rtThread = new Thread(osPriorityAboveNormal, 8 * 1024, NULL, "rtThread");
        osStatus threadstate = rtThread->start(callback(rtTask));
        if(threadstate != osOK) {
            printf("Failed starting rtThread\n");
        }

        CO_CANsetNormalMode(CO->CANmodule[0]);

        resetCmd = CO_RESET_NOT;
        uint16_t previousTicksMs = CO_timer1ms;
        
        printf("CANopen ready\nMovement init in progress\n");

        while(resetCmd == CO_RESET_NOT)
        {
            // подсчитать количество милисекунд с последнего запуска цикла
            uint16_t currentTicksMs = CO_timer1ms;
            uint16_t millisDiff = currentTicksMs - previousTicksMs;
            previousTicksMs = currentTicksMs;

            // асинхронные операции CANopen
            resetCmd = CO_process(CO, millisDiff, NULL);

            // Инициализация устройства
            if(!configured)
            {
                // данный узел должен быть в состоянии operational
                CO_NMT_setInternalState(CO->NMT, CO_NMT_OPERATIONAL);
                
                // отправляется SDO (канал связи, индекс объекта сервера, суб-индекс, данные, размер данных, флаг блоковой передачи)
                res =  CO_SDOclientDownloadInitiate(CO->SDOclient[0], MODE_OF_OPERATION, 0x00, (uint8_t *) &mode, sizeof(mode), 0);
                printf("Mode of operation sent, status %i\n", res);

                res =  CO_SDOclientDownloadInitiate(CO->SDOclient[0], PROFILE_VELOCITY, 0x00, (uint8_t *) &profile_velocity, sizeof(profile_velocity), 0);
                printf("Profile velocity sent, status %i\n", res);

                res =  CO_SDOclientDownloadInitiate(CO->SDOclient[0], PROFILE_ACCELERATION, 0x00, (uint8_t *) &profile_acceleration, sizeof(profile_acceleration), 0);
                printf("Profile acceleration sent, status %i\n", res);

                res =  CO_SDOclientDownloadInitiate(CO->SDOclient[0], PROFILE_DECELERATION, 0x00, (uint8_t *) &profile_deceleration, sizeof(profile_deceleration), 0);
                printf("Profile deceleration sent, status %i\n", res);

                res =  CO_SDOclientDownloadInitiate(CO->SDOclient[0], MOTION_PROFILE_TYPE, 0x00, (uint8_t *) &motion_profile_type, sizeof(motion_profile_type), 0);
                printf("Motion profile type sent sent, status %i\n", res);
                // проводится перезагрузка в соответствии с документацией
                res =  CO_SDOclientDownloadInitiate(CO->SDOclient[0], CONTROLWORD, 0x00, (uint8_t *) &controlword, sizeof(controlword), 0);
                printf("Performing restart, status %i\n", res);

                controlword = SWITCH_ON;
                res =  CO_SDOclientDownloadInitiate(CO->SDOclient[0], CONTROLWORD, 0x00, (uint8_t *) &controlword, sizeof(controlword), 0);
                printf("Enabling device, status %i\n", res);
                
                controlword = PPM_ABS_IMM; // это слово запускает движение

                printf("Movement init finished, start of cycle\n");

                // двигаться по часовой
                res =  CO_SDOclientDownloadInitiate(CO->SDOclient[0], TARGET_POSITION, 0x00, (uint8_t *) &alpha, sizeof(alpha), 0);
                printf("Target position alpha sent, status %i\n", res);

                res =  CO_SDOclientDownloadInitiate(CO->SDOclient[0], CONTROLWORD, 0x00, (uint8_t *) &controlword, sizeof(controlword), 0);
                printf("Controlword for movement start sent, status %i\n", res);

                configured = true;

                OD_readInput8Bit[0] = 0xff;

            }

            // пришел statusword с битом завершения движения
            if((OD_writeOutput16Bit[0] & STATUSWORD_FINISH_BIT) == STATUSWORD_FINISH_BIT){
                if(repeats!=0){
                    printf("Motion finished, writing new target position\n");

                    if (cw)
                    {        // движение по часовой
                        res =  CO_SDOclientDownloadInitiate(CO->SDOclient[0], TARGET_POSITION, 0x00, (uint8_t *) &alpha, sizeof(alpha), 0);
                        printf("Target position alpha sent, status %i\n", res);

                        res =  CO_SDOclientDownloadInitiate(CO->SDOclient[0], CONTROLWORD, 0x00, (uint8_t *) &controlword, sizeof(controlword), 0);
                        printf("Controlword for movement start sent, status %i\n", res);

                        cw = false; // следующее движение должно быть против часовой
                    }
                    else 
                    {         // движение против часовой
                        res =  CO_SDOclientDownloadInitiate(CO->SDOclient[0], TARGET_POSITION, 0x00, (uint8_t *) &beta, sizeof(alpha), 0);
                        printf("Target position beta sent, status %i\n", res);

                        res =  CO_SDOclientDownloadInitiate(CO->SDOclient[0], CONTROLWORD, 0x00, (uint8_t *) &controlword, sizeof(controlword), 0);
                        printf("Controlword for movement start sent, status %i\n", res);

                        cw = true; // следующее движение должно быть по часовой
                        repeats -= 1; // движение против часовой завершает итерацию цикла
                    }
                }
                else 
                {
                    if(!home)
                    {
                        // начать движение к стартовой позиции
                        printf("Cycle finished, returning to the start position\n");
                        res =  CO_SDOclientDownloadInitiate(CO->SDOclient[0], TARGET_POSITION, 0x00, (uint8_t *) &start, sizeof(start), 0);
                        printf("Target position of starting point sent, status %i\n", res);

                        res =  CO_SDOclientDownloadInitiate(CO->SDOclient[0], CONTROLWORD, 0x00, (uint8_t *) &controlword, sizeof(controlword), 0);
                        printf("Controlword for movement start sent, status %i\n", res);

                        home = true;
                    }
                    else
                    {
                        // отключить EPOS2
                        controlword = SHUTDOWN;
                        res =  CO_SDOclientDownloadInitiate(CO->SDOclient[0], CONTROLWORD, 0x00, (uint8_t *) &controlword, sizeof(controlword), 0);
                        printf("Controlword for shutdown sent, status %i\n", res);
                    }
                    
                }
                // очистить statusword чтобы предотвратить зацикливание
                OD_writeOutput16Bit[0] = 0x00;
            }


        }

        printf("CANopen communication reset\n");
        rtThread->flags_set(STOP_THREADS_FLAG);
        rtThread->join();
        delete(rtThread);
        rtThread = NULL;

        // убрать объекты из памяти
        CO_delete((void*)CAN_MODULE_ADDRESS); // может аварийно завершить приложение в случае ошибки

        // Сбросить подключения CANopen, очистить и восстановить словарь 
        CO_CANreset();
    }

    // выход из программы
    printf("Program exit!!\n");

    // сбросить
    if(resetCmd != CO_RESET_QUIT)
        system_reset();
    return 0;
}


static void rtLoop(void)
{
    // Обработать SYNC и RPDO
    bool_t syncWas = CO_process_SYNC(CO, RTTHREAD_INTERVAL_1000US);
    CO_process_RPDO(CO, syncWas);

    //Вывести RPDO на светодиод
    // выводится только первый бит RPDO-0
    uint16_t outputmap = OD_writeOutput16Bit[0];
    userLed.write(outputmap & 0x01);

    // применить цифровой вход к TPDO
    // кнопка записывает только первй бит TPDO-0 и инвертируется
    uint8_t inputmap = (userButton.read() == 0 ? 1 : 0) & 0x01;
    OD_readInput8Bit[0] = inputmap;

    // Обработать TPDO
    CO_process_TPDO(CO, syncWas, RTTHREAD_INTERVAL_1000US);

    //Обновить индикаторы
    CO_Indicators_process(CO->NMT);
}


// данная функция исполняется в фиксированные моменты времени
static void rtTask(void)
{
    rtPerfTimer.start();

    // Sleep ровно на 1 мс.
    // Данная операция планирует период неактивности данного потока, позволяя основной программе
    // продолжать исполнение.
    while(!ThisThread::flags_wait_any_for(STOP_THREADS_FLAG, RTTHREAD_INTERVAL_1000US/1000)) {

        CO_timer1ms++;

        if(CO->CANmodule[0]->CANnormal) {
            // Таймер используется для замера времени исполнения циклической процедуры
            // В случае плохой производительности будет содано EMCY сообщение позже
            rtPerfTimer.reset();
            int begin = rtPerfTimer.read_us();

            // выполнить петлю реального времени
            rtLoop();

            //подсчитать время исполнения, отправить EMCY в случае плохой производительности (>1ms)
            int timespan = rtPerfTimer.read_us() - begin;
            if(timespan > RTTHREAD_INTERVAL_1000US) {
                float tsMillis = static_cast<double>(timespan) / 1000;
                printf("RT slow: %.3fms\n", tsMillis);
                CO_errorReport(CO->em, CO_EM_ISR_TIMER_OVERFLOW, CO_EMC_SOFTWARE_INTERNAL, 0U); 
                // устройство перейдет в pre-op состояние и потребует перезагрузки
            }
        }
    }
}

// функция чтения из последовательного порта
char * read_from_serial()
{
    buffer[0] = '\0';
    int len = 0;
    bool read = true;
    while (read) 
    {
        char new_char = pc.getc(); 
        
        buffer[len] = new_char; 
        len+=1;         
    
        // сообщение завершено
        if (new_char == '\n') {
            read = false;
            buffer[len] = '\0';
        }
    }
    return buffer;
}