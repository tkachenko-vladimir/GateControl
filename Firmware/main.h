#define SAFE_DIST 50000
#define SPEED1_CAL 100
#define SPEED2_CAL 217
#define SPEED3_CAL 307
#define SPEED4_CAL 336
#define SPEED5_CAL 362
#define PED_DIST_OPEN 300000
#define ROLLBACK_DIST 30000
#define MIN_TESTBAT_PERIOD 300
#define TEST_BAT_LONG 3000

#define btn_open 1
#define btn_close 2
#define btn_stop 3
#define btn_s1 4
#define btn_s2 5
#define btn_pb 6
#define btn_ph1 7
#define btn_ph2 8
#define left_dir TRUE
#define right_dir FALSE
#define FLT_OTHER_SHORT 1
#define FLT_OTHER_BREAK 2

#define STOP_DRIVE {speed=0;while((M_BACK==1)||(M_FORW==1))OS_Yield();}
#define IS_NOT_CLOSE ((m_right && !C_LEFT) || (!m_right && !C_RIGHT))
#define IS_NOT_OPEN ((m_right && !C_RIGHT) || (!m_right && !C_LEFT))
#define IS_CLOSED ((m_right && C_LEFT) || (!m_right && C_RIGHT))

#define VBAT_SENSE PB_IDR_bit.IDR0
#define VCC_SENSE PB_IDR_bit.IDR1
#define I_SENSE PB_IDR_bit.IDR2
#define FIRE PB_IDR_bit.IDR3
#define U_BAT_SW PA_ODR_bit.ODR4
#define LD_BAT_SW PA_ODR_bit.ODR5
#define CHRG_SW PA_ODR_bit.ODR6
#define BTN_OPEN PA_IDR_bit.IDR3
#define BTN_CLOSE PB_IDR_bit.IDR4
#define BTN_STOP PB_IDR_bit.IDR5
#define C_LEFT PB_IDR_bit.IDR7
#define C_RIGHT PB_IDR_bit.IDR6
#define BTN_PB PE_IDR_bit.IDR7
#define PHOTO1 PE_IDR_bit.IDR6
#define PHOTO2 PE_IDR_bit.IDR5
#define M_RIGHT PG_IDR_bit.IDR0
#define NOFIRECONTROL PG_IDR_bit.IDR1
#define LOWSPEED PC_IDR_bit.IDR3
#define DIP4 PC_IDR_bit.IDR4
#define DIP5 PC_IDR_bit.IDR5
#define DIP6 PC_IDR_bit.IDR6
#define K_IN_FLT PC_ODR_bit.ODR7
#define K_BAT_FLT PE_ODR_bit.ODR0
#define OUT_FLT_OTHER PD_ODR_bit.ODR3
#define OUT_CLOSED PD_ODR_bit.ODR2
#define OUT_LIGHT PC_ODR_bit.ODR1
#define M_PWM PD_ODR_bit.ODR4
#define M_FORW PC_ODR_bit.ODR2
#define M_BACK PD_ODR_bit.ODR0
#define BTN1 PE_IDR_bit.IDR3
#define HL_BAT_OK PD_ODR_bit.ODR7

void Start(void);
void Task1(void);
void Task2(void);
void Task3(void);
void Task4(void);
void Task5(void);
void Task6(void);
void init_gpio(void);
void init_timers(void);
void init_adc(void);
void InitialiseIWDG(void);
bool safe_dist_chk(void);
void set_direction_open(void);
void set_direction_close(void);
void set_fullspeed(void);