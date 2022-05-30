#include "stm8s.h"
#include "iostm8s105c6.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "ctype.h"
#include "osa.h"
#include "stm8s_adc1.h"
#include "main.h"

const unsigned int speed_tab[4] = {50, 95, 170, 252};
//const unsigned int speed_tab[4] = {31, 95, 170, 252};
unsigned int speed_cur = 0, speed_set = 0, difference = 0;
char in_byte;
volatile unsigned int adc_buffer[3];
unsigned int Vcc_adc[4], Vbt_adc[4], I_adc[4], Fire_adc, Pb_adc;
unsigned char Vcc_adc_c = 0, Vbt_adc_c = 0, I_adc_c = 0, overcurrent_err_cnt = 0;
volatile unsigned int Vcc_adcS = 0, Vbt_adcS = 0, I_adcS = 0;
unsigned char speed = 0, cnt1 = 0;
bool calib = FALSE, calib_error = FALSE, direction = right_dir;
volatile unsigned int b_timer = 0;
volatile unsigned long timer1 = 0, timer2 = 0, test_bat_timer = 0;
volatile unsigned int timer3 = 0, bat_measuring_timer = 0;
unsigned long cur_pos = 0;//текущая позиция ворот, как время до полного открытия
unsigned long tmp_pos = 0;

bool b1 = FALSE, b2 = FALSE, b3 = FALSE, b4 = FALSE;
volatile unsigned char btn_code = 0, FLT_OTHER_F = 0, duty_cycle = 60;
unsigned char bt1 = 0, bt2 = 0, bt3 = 0, bt4 = 0;
volatile bool overcurrent = FALSE, FIRE_FLAG = FALSE, u_in_flt = FALSE, u_bat_flt = FALSE, fire_flt1 = FALSE, fire_flt2 = FALSE;
volatile bool bat_measuring = FALSE;//измерение напряжения батареи
volatile bool measuring_done = FALSE, test_bat_f = FALSE;
bool chrg_crrnt = FALSE, battery_busy = FALSE, photo_flt = FALSE;
unsigned int msec_cnt = 0, sec_cnt = 0, min_cnt = 0, hour_cnt = 0;
bool photo_f = FALSE;
unsigned int photo_cnt = 0;
unsigned char open_fire_cnt = 0;

#pragma location=0x004000
__no_init unsigned char eeprom_flag;
#pragma location=0x004001
__no_init unsigned long path_time;//время полного хода ворот, мсек
#pragma location=0x004010
__no_init bool m_right;

bool
safe_dist_chk(void)
{
  if((m_right && (direction == right_dir)) || (!m_right && (direction == left_dir)))//открываются
  {
    if(cur_pos < SAFE_DIST)
      return TRUE;
  }
  if((m_right && (direction == left_dir)) || (!m_right && (direction == right_dir)))//закрываются
  {
    if((path_time - cur_pos) < SAFE_DIST)
      return TRUE;
  }
  return FALSE;
}

void
set_direction_open(void)
{
  if(m_right)
    direction = right_dir;
  else
    direction = left_dir;
}

void
set_direction_close(void)
{
  if(m_right)
    direction = left_dir;
  else
    direction = right_dir;
}

void
set_fullspeed(void)
{
  if(LOWSPEED)
    speed = 3;
  else
    speed = 4;
}

int
main(void)
{
  CLK_ECKR_bit.HSEEN = 1;
  CLK_SWCR_bit.SWEN=1;
  while(CLK_ECKR_bit.HSERDY != 1);
  CLK_CKDIVR = 0;
  CLK_SWR = 0xB4;
  while(CLK_SWCR_bit.SWIF != 1);

  init_gpio();
  
  U_BAT_SW = 0;
  LD_BAT_SW = 0;
  CHRG_SW = 0;
  K_IN_FLT = 0;
  K_BAT_FLT = 0;
  OUT_FLT_OTHER = 0;
  OUT_CLOSED = 0;
  OUT_LIGHT = 0;
  HL_BAT_OK = 0;
  M_PWM = 0;
  M_FORW = 0;
  M_BACK = 0;
  chrg_crrnt = TRUE;

  OS_Init();
  init_timers();
  init_adc();
  
  FLASH_DUKR = 0xAE;
  FLASH_DUKR = 0x56; 
  if(eeprom_flag != 0xAD)
  {
    eeprom_flag = 0xAD;
    path_time = 860000;//время полного хода ворот
    if(M_RIGHT)
      m_right = FALSE;//мотор слева
    else
      m_right = TRUE;//мотор справа
  }

  __enable_interrupt();
  
  OS_Task_Create(0, Start);

  while(1)
  {
    OS_Run();
  };
}

void Start(void)
{
  HL_BAT_OK = 1;
  OS_Delay(500);
  HL_BAT_OK = 0;
  OS_Delay(500);
  HL_BAT_OK = 1;

  OS_Task_Create(0, Task1);
  OS_Task_Create(0, Task2);
  OS_Task_Create(0, Task3);
  OS_Task_Create(0, Task4);
  OS_Task_Create(0, Task5);
  OS_Task_Create(0, Task6);

  InitialiseIWDG();
  for(;;)
  {
    if(PHOTO1 || PHOTO2)//определение ошибки фотобаръеров
    {
      if(!photo_f)
      {
        photo_f = TRUE;
        photo_cnt = 120;//если фотобаръер показывает препятствие более 2 минут то ошибка
      }
      else
      {
        if(photo_cnt == 0)
          photo_flt = TRUE;
      }
    }
    else
    {
      photo_f = FALSE;
      photo_flt = FALSE;
    }

    if(fire_flt1 || fire_flt2 || photo_flt)//если ошибка пожарного шлейфа или фотобаръера то устанавливаем выход общей ошибки
      OUT_FLT_OTHER = 1;
    else
      OUT_FLT_OTHER = 0;
    if((m_right && C_LEFT) || (!m_right && C_RIGHT))//проверка концевика и вывод сигнала "ворота закрыты"
    {
      OUT_CLOSED = 1;
      cur_pos = path_time;
    }
    else
      OUT_CLOSED = 0;
    if((m_right && C_RIGHT) || (!m_right && C_LEFT))//проверка ворота полностью открыты для корректировки позиции
      cur_pos = 0;
    
    if((bat_measuring_timer == 0) && !battery_busy)//измерение напряжения батареи если она свободна
    {
      battery_busy = TRUE;//занимаем батарею
      chrg_crrnt = FALSE;//запрещаем зарядку
      CHRG_SW = 0;
      U_BAT_SW = 1;//слегка нагружаем батарею для снятия остаточного напряжения
      OS_Delay(300);
      bat_measuring = TRUE;//разрешаем измерение напряжения батареи
      measuring_done = FALSE;
      while(!measuring_done)
        OS_Yield();
      bat_measuring = FALSE;//запрещаем измерение напряжения батареи
      U_BAT_SW = 0;//снимаем нагрузку с батареи
      chrg_crrnt = TRUE;//разрешаем зарядку
      battery_busy = FALSE;//освобождаем батарею
      bat_measuring_timer = 10000;
    }
    
    if(test_bat_f && !battery_busy)//тестовая нагрузка батареи
    {
      battery_busy = TRUE;//занимаем батарею
      test_bat_f = FALSE;
      if(test_bat_timer == 0)//проверка разрешена не чаще 1 раз в 5 минут
      {
        chrg_crrnt = FALSE;//запрещаем зарядку
        CHRG_SW = 0;
        LD_BAT_SW = 1;//нагружаем
        bat_measuring = TRUE;//разрешаем измерение напряжения батареи
        timer3 = TEST_BAT_LONG;
        if(HL_BAT_OK)//индикация о начале проверки
          HL_BAT_OK = 0;
        else
          HL_BAT_OK = 1;
        u_bat_flt = FALSE;
        while((timer3 != 0) && !u_bat_flt)//пока таймер не закончится или не выставится ошибка
          OS_Yield();
        LD_BAT_SW = 0;//снимаем нагрузку
        if(timer3 == 0)//если тест пройден то снимаем сигнал ошибки
        {
          K_BAT_FLT = 0;
          HL_BAT_OK = 1;
        }
        bat_measuring = FALSE;//запрещаем измерение напряжения батареи
        chrg_crrnt = TRUE;//разрешаем зарядку
        test_bat_timer = MIN_TESTBAT_PERIOD;
      }
      else//если проверка сейчас запрещена то мигаем два раза
      {
        if(HL_BAT_OK)
        {
          HL_BAT_OK = 0;
          OS_Delay(300);
          HL_BAT_OK = 1;
          OS_Delay(300);
          HL_BAT_OK = 0;
          OS_Delay(300);
          HL_BAT_OK = 1;
        }
        else
        {
          HL_BAT_OK = 1;
          OS_Delay(300);
          HL_BAT_OK = 0;
          OS_Delay(300);
          HL_BAT_OK = 1;
          OS_Delay(300);
          HL_BAT_OK = 0;
        }
      }
      battery_busy = FALSE;//освобождаем батарею
    }

    OS_Delay(10);
  }
}

void Task1(void)
{
  for(;;)
  {
    if(FIRE_FLAG)//выбор режима работы по сигналу пожар
    {
      if(IS_NOT_CLOSE)//если ворота не закрыты то начать закрывать
        btn_code = btn_close;

      if(btn_code == btn_pb)//открытие
      {
        btn_code = 0;
        if(IS_NOT_OPEN)//если ворота уже не открыты или не исчерпано количество попыток
        {
          set_direction_open();
          set_fullspeed();
          while(IS_NOT_OPEN)//едем до концевика
          {
            if(safe_dist_chk())
              speed = 1;//конец таймера, переход на малую скорость
            if(btn_code == btn_stop)
            {
              btn_code = 0;
              break;
            }
            if(btn_code == btn_close)
            {
              break;
            }
            if(!FIRE_FLAG)
            {
              break;
            }
            if(overcurrent)//есть препятствие по превышению тока
            {
              STOP_DRIVE;
              overcurrent_err_cnt++;
              set_direction_close();
              speed = 1;
              tmp_pos = cur_pos;
              while((IS_NOT_CLOSE && ((cur_pos - tmp_pos) < ROLLBACK_DIST)))//едем ROLLBACK_DIST или до концевика
              {
                if(btn_code == btn_stop)
                {
                  break;
                }
                OS_Yield();
              }
              STOP_DRIVE;
              set_direction_open();
              set_fullspeed();
              if((overcurrent_err_cnt == 3) || (btn_code == btn_stop))
                break;
            }
            OS_Yield();
          }
          STOP_DRIVE;
          overcurrent_err_cnt = 0;
          open_fire_cnt++;
          if(open_fire_cnt == 1)//первое открытие на 10 секунд
          {
            timer2 = 10000;
            while(timer2 != 0)
            {
              if(btn_code == btn_close)
              {
                break;
              }
              OS_Yield();
            }
          }
          else//последующие открытия на 30 секунд
          {
            timer2 = 30000;
            while(timer2 != 0)
            {
              if(btn_code == btn_close)
              {
                break;
              }
              OS_Yield();
            }
          }
        }
      }
      
      if(btn_code == btn_close)//закрытие
      {
        btn_code = 0;
        if(IS_NOT_CLOSE)//если ворота уже не закрыты
        {
          set_direction_close();
          set_fullspeed();
          while(IS_NOT_CLOSE)//едем до концевика
          {
            if(safe_dist_chk())
              speed = 1;//конец таймера, переход на малую скорость
            if(btn_code == btn_stop)
            {
              btn_code = 0;
              break;
            }
            if(btn_code == btn_open)
            {
              break;
            }
            if(!FIRE_FLAG)
            {
              break;
            }
            if(PHOTO1 || PHOTO2)//есть препятствие по фотодатчику
            {
              STOP_DRIVE;
              while(PHOTO1 || PHOTO2)
              {
                OS_Yield();
              }
              set_direction_close();
              set_fullspeed();
            }
            if(overcurrent)//есть препятствие по превышению тока
            {
              STOP_DRIVE;
              set_direction_open();
              speed = 1;
              tmp_pos = cur_pos;
              while((IS_NOT_OPEN && ((tmp_pos - cur_pos) < ROLLBACK_DIST)))//едем ROLLBACK_DIST или до концевика
              {
                OS_Yield();
              }
              STOP_DRIVE;
              set_direction_close();
              set_fullspeed();
            }
            OS_Yield();
          }
          STOP_DRIVE;
        }
      }
      
      if(btn_code == btn_stop)//стоп
      {
        btn_code = 0;
      }
    }
    else
    {
      open_fire_cnt = 0;//сбрасываем количество открытий при пожаре
      if(btn_code == btn_pb)//частичное открытие
      {
        btn_code = 0;
        if(IS_CLOSED)//если ворота закрыты
        {
          set_direction_open();
          set_fullspeed();
          while(IS_NOT_OPEN && ((path_time - cur_pos) < PED_DIST_OPEN))//едем PED_DIST_OPEN или до концевика
          {
            if(FIRE_FLAG)
            {
              break;
            }
            if(overcurrent)//есть препятствие по превышению тока
            {
              STOP_DRIVE;
              overcurrent_err_cnt++;
              set_direction_close();
              speed = 1;
              tmp_pos = cur_pos;
              while((IS_NOT_CLOSE && ((cur_pos - tmp_pos) < ROLLBACK_DIST)))//едем ROLLBACK_DIST или до концевика
              {
                if(btn_code == btn_stop)
                {
                  break;
                }
                OS_Yield();
              }
              STOP_DRIVE;
              set_direction_open();
              set_fullspeed();
              if((overcurrent_err_cnt == 3) || (btn_code == btn_stop))
                break;
            }
            OS_Yield();
          }
          STOP_DRIVE;
          overcurrent_err_cnt = 0;
        }
      }

      if(btn_code == btn_open)//открытие
      {
        btn_code = 0;
        if(IS_NOT_OPEN)//если ворота уже не открыты
        {
          set_direction_open();
          set_fullspeed();
          while(IS_NOT_OPEN)//едем до концевика
          {
            if(safe_dist_chk())
              speed = 1;//конец таймера, переход на малую скорость
            if(btn_code == btn_stop)
            {
              btn_code = 0;
              break;
            }
            if(btn_code == btn_close)
            {
              break;
            }
            if(FIRE_FLAG)
            {
              break;
            }
            if(overcurrent)//есть препятствие по превышению тока
            {
              STOP_DRIVE;
              overcurrent_err_cnt++;
              set_direction_close();
              speed = 1;
              tmp_pos = cur_pos;
              while((IS_NOT_CLOSE && ((cur_pos - tmp_pos) < ROLLBACK_DIST)))//едем ROLLBACK_DIST или до концевика
              {
                if(btn_code == btn_stop)
                {
                  break;
                }
                OS_Yield();
              }
              STOP_DRIVE;
              set_direction_open();
              set_fullspeed();
              if((overcurrent_err_cnt == 3) || (btn_code == btn_stop))
                break;
            }
            OS_Yield();
          }
          STOP_DRIVE;
          overcurrent_err_cnt = 0;
        }
      }

      if(btn_code == btn_close)//закрытие
      {
        btn_code = 0;
        if(IS_NOT_CLOSE)//если ворота уже не закрыты
        {
          set_direction_close();
          set_fullspeed();
          while(IS_NOT_CLOSE)//едем до концевика
          {
            if(safe_dist_chk())
              speed = 1;//конец таймера, переход на малую скорость
            if(btn_code == btn_stop)
            {
              btn_code = 0;
              break;
            }
            if(btn_code == btn_open)
            {
              break;
            }
            if(FIRE_FLAG)
            {
              break;
            }
            if(PHOTO1 || PHOTO2)//есть препятствие по фотодатчику
            {
              STOP_DRIVE;
              set_direction_open();
              set_fullspeed();
              while(IS_NOT_OPEN && (PHOTO1 || PHOTO2))//едем до концевика или пока препятствие не уберется
              {
                if(safe_dist_chk())
                  speed = 1;//конец таймера, переход на малую скорость
                if(btn_code == btn_stop)
                {
                  btn_code = 0;
                  break;
                }
                OS_Yield();
              }
              STOP_DRIVE;
              while(PHOTO1 || PHOTO2)
              {
                OS_Yield();
              }
              set_direction_close();
              set_fullspeed();
            }
            if(overcurrent)//есть препятствие по превышению тока
            {
              STOP_DRIVE;
              overcurrent_err_cnt++;
              set_direction_open();
              speed = 1;
              tmp_pos = cur_pos;
              while((IS_NOT_OPEN && ((tmp_pos - cur_pos) < ROLLBACK_DIST)))//едем ROLLBACK_DIST или до концевика
              {
                if(btn_code == btn_stop)
                {
                  break;
                }
                OS_Yield();
              }
              STOP_DRIVE;
              set_direction_close();
              set_fullspeed();
              if((overcurrent_err_cnt == 3) || (btn_code == btn_stop))
                break;
            }
            OS_Yield();
          }
          STOP_DRIVE;
          overcurrent_err_cnt = 0;
        }
      }
      
      if(btn_code == btn_stop)//стоп
      {
        btn_code = 0;
      }

      if(calib)
      {
        if(M_RIGHT)//считываем значения дип-переключателей
          m_right = FALSE;
        else
          m_right = TRUE;
        calib_error = FALSE;

        if(IS_NOT_OPEN)//если ворота уже не открыты
        {
          set_direction_open();
          speed = 2;
          while(IS_NOT_OPEN)//едем до концевика
          {
            if(!calib || (btn_code == btn_stop) || overcurrent || PHOTO1 || PHOTO2)
            {
              calib_error = TRUE;
              break;
            }
            OS_Yield();
          }
          STOP_DRIVE;
        }
        if(!calib_error)
        {
          set_direction_close();
          speed = 2;
          timer1 = 0;
          cur_pos = 0;
          while(IS_NOT_CLOSE)//едем до концевика
          {
            if(!calib || (btn_code == btn_stop) || overcurrent || PHOTO1 || PHOTO2)
            {
              calib_error = TRUE;
              break;
            }
            OS_Yield();
          }
          if(!calib_error)
            path_time = cur_pos;
          STOP_DRIVE;
        }
        calib = FALSE;
      }
    }
    OS_Yield();
  }
}

void Task2(void)
{
  for(;;)
  {
    if(speed != 0)
    {
      OUT_LIGHT = 1;
      OS_Delay(500);
      OUT_LIGHT = 0;
      OS_Delay(500);
    }
    OS_Yield();
  }
}

void Task3(void)
{
  for(;;)
  {
    if(speed != 0)
    {
      if(!speed_set)
      {
        if(speed_cur == 0)
        {
          if(direction == left_dir)
          {
            M_BACK = 1;
            M_FORW = 0;
          }
          else
          {
            M_BACK = 0;
            M_FORW = 1;
          }
          OS_Delay(500);
        }
      }
    }
    switch(speed)
    {
    case 0:
      speed_set = 0;
      break;
    case 1:
      speed_set = speed_tab[0];
      if((m_right && (direction == right_dir)) || (!m_right && (direction == left_dir)) && (cur_pos > SPEED1_CAL))
        cur_pos = cur_pos - SPEED1_CAL;
      if((m_right && (direction == left_dir)) || (!m_right && (direction == right_dir)))
        cur_pos = cur_pos + SPEED1_CAL;
      break;
    case 2:
      speed_set = speed_tab[1];
      if((m_right && (direction == right_dir)) || (!m_right && (direction == left_dir)) && (cur_pos > SPEED1_CAL))
        cur_pos = cur_pos - SPEED2_CAL;
      if((m_right && (direction == left_dir)) || (!m_right && (direction == right_dir)))
        cur_pos = cur_pos + SPEED2_CAL;
      break;
    case 3:
      speed_set = speed_tab[2];
      if((m_right && (direction == right_dir)) || (!m_right && (direction == left_dir)) && (cur_pos > SPEED1_CAL))
        cur_pos = cur_pos - SPEED3_CAL;
      if((m_right && (direction == left_dir)) || (!m_right && (direction == right_dir)))
        cur_pos = cur_pos + SPEED3_CAL;
      break;
    case 4:
      speed_set = speed_tab[3];
      if((m_right && (direction == right_dir)) || (!m_right && (direction == left_dir)) && (cur_pos > SPEED1_CAL))
        cur_pos = cur_pos - SPEED4_CAL;
      if((m_right && (direction == left_dir)) || (!m_right && (direction == right_dir)))
        cur_pos = cur_pos + SPEED4_CAL;
      break;
    case 5:
      speed_set = 350;
      if((m_right && (direction == right_dir)) || (!m_right && (direction == left_dir)) && (cur_pos > SPEED1_CAL))
        cur_pos = cur_pos - SPEED5_CAL;
      if((m_right && (direction == left_dir)) || (!m_right && (direction == right_dir)))
        cur_pos = cur_pos + SPEED5_CAL;
      break;
    }
    if(speed_set != speed_cur)
    {
      if(speed_set > speed_cur)
        speed_cur++;
      else
      {
        if(speed_cur > 5)
          speed_cur = speed_cur - 5;
        else
          speed_cur = 0;
        if(speed_cur == 0)
        {
          OS_Delay(500);
          M_BACK = 0;
          M_FORW = 0;
        }
      }
    }
    TIM2_CCR1H = (unsigned char)(speed_cur >> 8);
    TIM2_CCR1L = (unsigned char)speed_cur;
    OS_Delay(10);
  }
}

void Task4(void)
{
  for(;;)
  {
    if(!(BTN1))
    {
      OS_Delay(20);
      b_timer = 2000;
      while(!(BTN1) && b_timer)
        OS_Yield();
      if(b_timer)
      {
        if(calib)
          calib = FALSE;
        else
          test_bat_f = TRUE;
      }
      else
      {
        calib = TRUE;
        while(!(BTN1))
          OS_Yield();
      }
    }
    OS_Delay(20);
  }
}

void Task5(void)
{
  for(;;)
  {
    bt1 = bt1 << 1;
    if(!(BTN_OPEN))
      bt1++;
    bt2 = bt2 << 1;
    if(!(BTN_CLOSE))
      bt2++;
    bt3 = bt3 << 1;
    if(!(BTN_STOP))
      bt3++;

    if(((bt1 & 3) == 3) && !b1)
    {
      b1 = TRUE;
      btn_code = btn_open;
    }
    if(((bt2 & 3) == 3) && !b2)
    {
      b2 = TRUE;
      btn_code = btn_close;
    }
    if(((bt3 & 3) == 3) && !b3)
    {
      b3 = TRUE;
      btn_code = btn_stop;
    }

    if((bt1 & 3) == 0)
      b1 = FALSE;
    if((bt2 & 3) == 0)
      b2 = FALSE;
    if((bt3 & 3) == 0)
      b3 = FALSE;

    OS_Delay(10);
  }
}

void Task6(void)
{
  for(;;)
  {
    if(!u_in_flt)//если есть внешнее напряжение
    {
      for(cnt1 = 0; cnt1 < 100; cnt1++)
      {
        if(chrg_crrnt)//если зарядка разрешена
        {
          if(cnt1 > duty_cycle)
            CHRG_SW = 1;
          else
            CHRG_SW = 0;
        }
        OS_Delay(2);
      }

      difference =  28500 - Vbt_adcS; 
      if(Vbt_adcS > 27000)
        duty_cycle = 100;
      else
        duty_cycle = 0;
      /*
      if(difference > 18000)
        duty_cycle = 5;
      if((difference > 17000) && (difference <= 18000))
        duty_cycle = 7;
      if((difference > 15000) && (difference <= 17000))
        duty_cycle = 10;
      if((difference > 13500) && (difference <= 15000))
        duty_cycle = 13;
      if((difference > 12000) && (difference <= 13500))
        duty_cycle = 17;
      if((difference > 10000) && (difference <= 12000))
        duty_cycle = 20;
      if((difference > 8500) && (difference <= 10000))
        duty_cycle = 23;
      if((difference > 6500) && (difference <= 8500))
        duty_cycle = 30;
      if((difference > 5000) && (difference <= 6500))
        duty_cycle = 45;
      if((difference > 3500) && (difference <= 5000))
        duty_cycle = 60;
      if((difference > 2500) && (difference <= 3500))
        duty_cycle = 80;
      if(difference <= 2500)
        duty_cycle = 100;
      */
    }
    else
      CHRG_SW = 0;
    
    OS_Yield();
    IWDG_KR = 0xAA;
  }
}

INTERRUPT_HANDLER(TIM4_handler, ITC_IRQ_TIM4_OVF)//1mS
{
  TIM4_SR_bit.UIF = 0;
  OS_Timer();
  if(b_timer != 0)
    b_timer--;
  timer1++;
  if(timer2 != 0)
    timer2--;
  if(timer3 != 0)
    timer3--;
  if(bat_measuring_timer != 0)
    bat_measuring_timer--;
  msec_cnt++;
  if(msec_cnt == 1000)
  {
    msec_cnt = 0;
    sec_cnt++;
    if(photo_cnt != 0)
      photo_cnt--;
    if(test_bat_timer != 0)
      test_bat_timer--;
    if(sec_cnt == 60)
    {
      sec_cnt = 0;
      min_cnt++;
      if(min_cnt == 60)
      {
        min_cnt = 0;
        hour_cnt++;
        if(hour_cnt == 24)
        {
          hour_cnt = 0;
          test_bat_f = TRUE;
        }
      }
    }
  }
}

INTERRUPT_HANDLER(UART2_RX, 21)
{
  if(UART2_SR_bit.OR_LHE)
  {
    UART2_SR_bit.OR_LHE = 0;
    UART2_SR_bit.RXNE = 0;
  }
  else
  {
    in_byte = UART2_DR;
  }
}

INTERRUPT_HANDLER(ADC1_IRQHandler, 22)
{
 ADC1_ClearITPendingBit(ADC1_IT_EOC);

 if(ADC_CSR_CH == 0)
 {
   if(bat_measuring)
   {
     Vbt_adc[Vbt_adc_c] = ADC1_GetConversionValue();
     Vbt_adc_c++;
     if(Vbt_adc_c == 4)
     {
       Vbt_adc_c = 0;
       Vbt_adcS = (Vbt_adc[0] + Vbt_adc[1] + Vbt_adc[2] + Vbt_adc[3]) / 4;
       Vbt_adcS = Vbt_adcS * 85;
       if(Vbt_adcS < 22000)
       {
         u_bat_flt = TRUE;
         K_BAT_FLT = 1;
         HL_BAT_OK = 0;
       }
       measuring_done = TRUE;
     }
   }
   else
     Vbt_adc_c = 0;
   ADC_CSR_CH = 1;
 }
 else
 {
   if(ADC_CSR_CH == 1)
   {
     Vcc_adc[Vcc_adc_c] = ADC1_GetConversionValue();
     Vcc_adc_c++;
     if(Vcc_adc_c == 4)
     {
       Vcc_adc_c = 0;
       Vcc_adcS = (Vcc_adc[0] + Vcc_adc[1] + Vcc_adc[2] + Vcc_adc[3]) / 4;
       Vcc_adcS = Vcc_adcS * 85;
       if(Vcc_adcS < 24000)
       {
         u_in_flt = TRUE;
         K_IN_FLT = 1;
       }
       else
       {
         u_in_flt = FALSE;
         K_IN_FLT = 0;
       }
     }
     ADC_CSR_CH = 2;
   }
   else
   {
     if(ADC_CSR_CH == 2)
     {
       I_adc[I_adc_c] = ADC1_GetConversionValue();
       I_adc_c++;
       if(I_adc_c == 4)
       {
         I_adc_c = 0;
         I_adcS = (I_adc[0] + I_adc[1] + I_adc[2] + I_adc[3]) / 4;
         I_adcS = I_adcS * 100;
         if(DIP4 && DIP5 && DIP6)//3A
         {
           if(I_adcS > 3000)
             overcurrent = TRUE;
           else
             overcurrent = FALSE;
         }
         if(DIP4 && DIP5 && !DIP6)//4A
         {
           if(I_adcS > 4000)
             overcurrent = TRUE;
           else
             overcurrent = FALSE;
         }
         if(DIP4 && !DIP5 && DIP6)//5A
         {
           if(I_adcS > 5000)
             overcurrent = TRUE;
           else
             overcurrent = FALSE;
         }
         if(DIP4 && !DIP5 && !DIP6)//6A
         {
           if(I_adcS > 6000)
             overcurrent = TRUE;
           else
             overcurrent = FALSE;
         }
         if(!DIP4 && DIP5 && DIP6)//7A
         {
           if(I_adcS > 7000)
             overcurrent = TRUE;
           else
             overcurrent = FALSE;
         }
         if(!DIP4 && DIP5 && !DIP6)//8A
         {
           if(I_adcS > 8000)
             overcurrent = TRUE;
           else
             overcurrent = FALSE;
         }
         if(!DIP4 && !DIP5 && DIP6)//9A
         {
           if(I_adcS > 9000)
             overcurrent = TRUE;
           else
             overcurrent = FALSE;
         }
         if(!DIP4 && !DIP5 && !DIP6)//10A
         {
           if(I_adcS > 10000)
             overcurrent = TRUE;
           else
             overcurrent = FALSE;
         }
       }
       ADC_CSR_CH = 3;
     }
     else
     {
       if(ADC_CSR_CH == 3)
       {
         Fire_adc = ADC1_GetConversionValue();
         ADC_CSR_CH = 8;
         if(NOFIRECONTROL)
         {
           if(Fire_adc < 512)
             FIRE_FLAG = FALSE;
           else
             FIRE_FLAG = TRUE;
           fire_flt1 = FALSE;
         }
         else
         {
           if(Fire_adc < 256)
             fire_flt1 = TRUE;
           else
           {
             if(Fire_adc < 644)
             {
               fire_flt1 = FALSE;
               FIRE_FLAG = FALSE;
             }
             else
             {
               if(Fire_adc < 900)
               {
                 fire_flt1 = FALSE;
                 FIRE_FLAG = TRUE;
               }
               else
                 fire_flt1 = TRUE;
             }
           }
         }
       }
       else
       {
         if(ADC_CSR_CH == 8)
         {
           Pb_adc = ADC1_GetConversionValue();
           ADC_CSR_CH = 0;
           bt4 = bt4 << 1;
           if(NOFIRECONTROL)
           {
             if(Pb_adc > 512)
               bt4++;
             fire_flt2 = FALSE;
           }
           else
           {
             if(Pb_adc < 256)
               fire_flt2 = TRUE;
             else
             {
               if(Pb_adc < 644)
               {
                 fire_flt2 = FALSE;
               }
               else
               {
                 if(Pb_adc < 900)
                 {
                   fire_flt2 = FALSE;
                   bt4++;
                 }
                 else
                   fire_flt2 = TRUE;
               }
             }
           }
           if(((bt4 & 3) == 3) && !b4)
           {
             b4 = TRUE;
             btn_code = btn_pb;
           }
           if((bt4 & 3) == 0)
             b4 = FALSE;
         }
       }
     }
   }
 }
 ADC1_StartConversion();
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
    nop();
  }
}
#endif