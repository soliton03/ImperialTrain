/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in try of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "fw_build.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* 製品版: 以下の診断マクロはすべて無効 */
//#define SOUND_DEBUG     /* サウンド再生/停止/切替のUART診断 */
//#define SOUND_TEST_MENU /* 音源確認メニュー(USART1): 1起動 2走行 3ラジエータ 0停止 */
//#define DIR_DEBUG       /* 尾灯方向/Power心拍のUART診断 */
//#define QA_RX_TRACE   /* [pop/skip/pair/drop/deb 018 m=0 v=2] 形式の診断ログ */
//#define CMD_DEBUG
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

//=====================================
// 電圧スレッシュホールド
//=====================================
#define POWER_VOLT_LOG	/* 1秒ごとに入力電圧をUART表示（始動診断用） */

/* しきい値は ADC 読取電圧(mV)。実電圧はブリッジ等の降下分を含む。
 * 校正: 実6.5V→5300, 実6.0V→4700 (GetPower_mV 換算値) */
#define POWER_ON_TH		((5000 * 5300) / 6500)	/* 実5.0V超 ≒ 読取4076mV */
#define POWER_OFF_TH	((4000 * 4700) / 6000)	/* 実4.0V未満 ≒ 読取3133mV */

/* 尾灯方向: 電源OFFを挟んだときだけ更新（ベル極性フリップは無視）。
 * 低電圧運転（実測 Power 読取 ~2.5V）でも武装・復帰できるよう
 * ON/OFF しきい値は 2.5V 未満に置く。逆転は主に peak からの落下でも検知。 */
#define DIR_POWER_OFF_MV      1200 /* deep OFF / reverse gap */
#define DIR_POWER_ON_MV       1800 /* solid ON (must be < typical run ~2500) */
#define DIR_POWER_OFF_MS         2U /* >=2ms below OFF_MV to arm */
#define DIR_DROP_ARM_MV        600 /* low-V minimum peak-to-now drop to arm */
#define DIR_DROP_ARM_PCT        40U /* low-V only: also require ~40% drop from peak */
#define DIR_DROP_ARM_MAX_PEAK 15000 /* peak追従あり: 高電圧でも短ギャップ反転をDROPで取る */
#define DIR_VM_CHANGE_MS        40U /* VM stable window (PowerON / armed latch) */
/* 電源が落ちない手動反転: Quantum短パルスより長くVMが安定したらDIRを追従。
 * 汽笛はVMだけ反転しVinはほぼ一定 → Vin振れが無いときは追従しない。 */
#define DIR_VM_FOLLOW_MS       500U /* > PW_WHISTLE(264): 汽笛検出が先 */
#define DIR_QA_HOLD_MS         800U /* エッジ/汽笛/コマンド後の追従禁止 */
#define DIR_FOLLOW_VIN_SWING_MV 500 /* 直近窓でこれ以上Vinが振れていないと追従しない */
#define DIR_FOLLOW_VIN_WIN_MS  300U
#define DIR_FOLLOW_VIN_KEEP_MS 2000U /* 振れ検出後、hold+followが終わるまで保持 */
/* After power-good while armed:
 *  prev=FWD → wait VM=1 (REV). Do NOT force while VM still 0.
 *  prev=REV → take VM=0 early as FWD (late VM=1 is a lie on return). */
#define DIR_FWD_FORCE_MS      1000U /* was REV: VM=0でもすぐFWDに戻さない */
#define DIR_REV_FORCE_MS       600U /* was FWD: wait longer for VM=1 */
#define DIR_REV_FORCE_HARD_MS  900U /* last resort even if VM still 0 */
#define DIR_ON_SETTLE_MS       500U /* 方向確定後、再武装まで待つ */
#define DIR_POST_HOLD_MS      1500U /* 確定後のVM追従禁止 */
#define POWER_ADC_PERIOD_MS      1U /* 1ms: do not miss short power gap */

/* 音源フェーズ (CH0) */
#define PHRASE_FAN_LOOP    0   /* お召列車ラジエータファンループ */
#define PHRASE_RUN_LOOP    1   /* 走行音 */
#define PHRASE_STARTUP     2   /* 起動音 */

#define RUN_FAN_TEST_SHORT   /* テスト: 30秒毎（本番はコメントアウトして5分） */
#ifdef RUN_FAN_TEST_SHORT
#define RUN_FAN_INTERVAL_MS  (30u * 1000u)      /* テスト: 30秒毎 */
#else
#define RUN_FAN_INTERVAL_MS  (5u * 60u * 1000u) /* 5分毎にラジエター重ね */
#endif
#define RUN_FAN_DURATION_MS  (30u * 1000u)     /* ラジエター重ね 30秒 */
#define RUN_FAN_FADE_MS      (2u * 1000u)      /* フェードイン/アウト各2秒 */
#define CH_RUN               0
#define CH_FAN               1

typedef enum {
  runMainLoop,  /* 走行音のみ（インターバル待ち） */
  runFanLoop    /* 走行音 + ラジエター重ね */
} RunSoundMode;

static RunSoundMode RunSoundState = runMainLoop;
static uint32_t RunModeSince = 0;
/* ラジエターCH相対音量 0=無音 … 255=マスターと同じ（_SetCVolAll が参照） */
static uint8_t FanVolScale = 0;
static int FanCountdownLastSec = -1;

enum enPowerState{
	powerIdle,powerStart,powerON,powerStop
};

int PowerState=powerIdle;
static int LastPowerState=-1;

static const char * const PowerStateName[]={
	"Idle State","Start State","Run State","StopState"
};


/* ====== 移植：定数（Arduino相当） ====== */
#define TIMER_INTERVAL   1    // ms
#define PW_MIN           16
#define PW_CHK           48
#define PW_LONG          72
#define PW_BELL_MIN      128
#define PW_BELL_MAX      256
#define PW_WHISTLE       264
#define CBITLEN          11

#define CMD_WHISTLE_ON   0xF001
#define CMD_WHISTLE_OFF  0xF000
#define CMD_BELL         0xF002

#define ON	1
#define OFF	0

//極性判定回路用==========================
/* VMがPW_LONG(72ms)超えて同一→極性候補。尾灯反映は電源OFF(DIR_POWER_*)を挟んだときだけ */
volatile int IsNormalDir=true;	//true=進行方向, false=逆方向(尾灯点灯)
static volatile uint8_t DirLastVM=0;
static volatile uint16_t DirHoldMs=0;
static volatile uint16_t DirPowerOffMs=0;
static volatile uint8_t DirRelatchArmed=0;
static volatile uint8_t DirPowerOk=1; /* 0=電源OFF相当 */
static uint16_t DirRecoverMs=0;
static uint8_t DirPowerWasOff=0;
static uint8_t DirVmCand=0xFFu;   /* 復帰後の VM 候補 */
static uint16_t DirVmStableMs=0;  /* VM 候補が続いている時間 */
static uint16_t DirPowerPeakMv=0; /* recent peak for drop-arm */
static uint8_t DirArmLogged=0;
static uint8_t DirAllowArm=0; /* need solid ON before arming */
static uint8_t DirHadSolidOn=0; /* 1 after Power has been good at least once */
static uint8_t DirPostChangeLock=0; /* 1: block arm after CHANGE until ON settled */
static uint16_t DirOnSettleMs=0;
static uint8_t DirVmPowerGood=0; /* hyst: set @ON_MV, clear @OFF_MV */
static uint8_t DirInitPending=1; /* 1: next solid ON latches absolute DIR from VM */
static uint8_t DirInitCand=0xFFu;
static uint16_t DirInitStableMs=0;
static uint8_t DirFollowCand=0xFFu;
static uint16_t DirFollowStableMs=0;
static volatile uint32_t DirQaHoldUntil=0; /* この時刻までVM追従禁止 */
static uint16_t DirVinWinMax=0;
static uint16_t DirVinWinMin=0xFFFFu;
static uint16_t DirVinWinMs=0;
static uint8_t DirVinSwingOk=0; /* 1: 直近に十分なVin振れあり */
static uint32_t DirVinSwingUntil=0; /* この時刻まで振れOKを保持 */
static uint8_t DirMeasEnable=0; /* 1: log power/VM edges with timestamps */
static uint32_t DirMeasT0=0;
static int8_t DirMeasLastPwr=-1; /* -1 unk, 0 off-like, 1 on */
static int8_t DirMeasLastVm=-1;
static uint8_t DirVmWatchEnable=0; /* 1: print VMIN edges when Power is OK */
static int8_t DirVmWatchLast=-1;
static uint8_t DirVmWatchBrownout=0; /* 1: already noted brownout this dip */
#ifdef DIR_DEBUG
static uint32_t DirPowerLogMs=0; /* last periodic Power log */
static uint8_t DirWaitLogged=0;
#endif
#ifdef POWER_VOLT_LOG
static uint32_t PowerVoltLogMs=0;
static int DirLogLastVm=-1;
static uint32_t DirLogVmTick=0;
#endif
static int LastLoggedDir=-1;

typedef enum {
  QS_IDLE  = 0,
  QS_SET   = 1,
  QS_RESET = 2
} QaState;

//音量調整用========================================
/* 旧4段階: OFF(127), 小(64), 中(32), 大(0) — CVOL 生値
 * 現在3段階: 小・中・大のみ（OFF相当は段階に含めない）。起動時は大(2)。
 * MUTE 時のみ旧 OFF と同じ 127 を適用。 */
static uint8_t IsMuted = 0;
int cVolume=2;
static const int VolumeTbl[3]={64,32,0};
static uint32_t VolUpGuardUntil = 0;

void _SetCVolAll(uint8_t cv);
void _SetVolume(void);

static void SetMute(uint8_t muteOn)
{
  IsMuted = muteOn ? 1u : 0u;
  _SetVolume();
}

void _SetVolume(){
	int vol;
	if(IsMuted != 0){
		vol=127;
	}else{
		vol=VolumeTbl[cVolume];
	}
	_SetCVolAll((uint8_t)vol);
}
void VolumeUp(){
	if(cVolume<2){
		cVolume++;
	}
	_SetVolume();
}

void VolumeDown(){
	if(cVolume>0){
		cVolume--;
	}
	_SetVolume();
}


/* ====== GPIO ヘルパ（VP_PIN→VPIN, VM_PIN→VMIN, 4/5→IO0/IO1） ======
 *  ※ 以下のマクロは CubeMX が自動生成する main.h 由来のものを利用します。
 *    VPIN_GPIO_Port / VPIN_Pin
 *    VMIN_GPIO_Port / VMIN_Pin
 *    IO0_GPIO_Port  / IO0_Pin   // 旧 Arduino pin 4
 *    IO1_GPIO_Port  / IO1_Pin   // 旧 Arduino pin 5
 */
static inline int digitalRead_VP(void){
  return HAL_GPIO_ReadPin(VPIN_GPIO_Port, VPIN_Pin) ? 1 : 0;
}
static inline int digitalRead_VM(void){
  return HAL_GPIO_ReadPin(VMIN_GPIO_Port, VMIN_Pin) ? 1 : 0;
}
/* VM=1 → 反転(REV/尾灯ON), VM=0 → 正転(FWD/尾灯OFF) */
static inline int DirFromVm(int vm)
{
  return vm ? 0 : 1;
}
static inline void digitalWrite_IO0(int level){
  HAL_GPIO_WritePin(IO0_GPIO_Port, IO0_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
/* isNormalDir: true=進行方向(消灯), false=逆方向(点灯). nTailLEDはActive-L */
static inline void SetTailLED(int isNormalDir){
  /* Active-L: FWD(消灯)=High, REV(点灯)=Low */
  HAL_GPIO_WritePin(nTailLED_GPIO_Port, nTailLED_Pin,
                    isNormalDir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline int TailLedGpioIsHigh(void){
  return (HAL_GPIO_ReadPin(nTailLED_GPIO_Port, nTailLED_Pin) == GPIO_PIN_SET) ? 1 : 0;
}

/* Apply direction to LED and log what the pin actually is. */
static void DirApplyTailLed(const char *why)
{
  SetTailLED(IsNormalDir);
#ifdef DIR_DEBUG
  printf("TAIL %s (%s) nTailLED=%s gpio=%d\n",
         IsNormalDir ? "OFF" : "ON",
         why,
         IsNormalDir ? "H(off)" : "L(on)",
         TailLedGpioIsHigh());
#else
  (void)why;
#endif
}

static inline void digitalWrite_IO2(int level){
  HAL_GPIO_WritePin(IO2_GPIO_Port, IO2_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
/* ====== コマンドキュー（ISR安全な最小リングバッファ） ====== */
#define CMDQ_SIZE 16
static volatile int      cmdq[CMDQ_SIZE];
static volatile uint8_t  q_head = 0, q_tail = 0;
static inline bool cmdq_is_empty(void){ return q_head == q_tail; }
static inline void cmdq_push_isr(int v){
  uint8_t nh = (uint8_t)(q_head + 1);
  if (nh != q_tail) { cmdq[q_head] = v; q_head = nh; } // フル時は捨てる
}
static inline int cmdq_pop(void){
  if (cmdq_is_empty()) return -1;
  int v = cmdq[q_tail]; q_tail = (uint8_t)(q_tail + 1); return v;
}
static inline int cmdq_peek(void)
{
  if (cmdq_is_empty()) return -1;
  return cmdq[q_tail];
}

/* ====== 変数（Arduinoスケッチ由来） ====== */
static volatile uint8_t  lVM = 0; // 直近サンプル
static volatile uint8_t  VM  = 0; // ノイズキャンセル後
static volatile int      LastState   = QS_IDLE;	//実際には存在しないが便宜上入れておく
static volatile uint16_t PulseWidth  = 0;
static volatile bool     CmdMode     = false;
static volatile bool     CmdFrameArmed = false;
static volatile bool     IsWhistle   = false;
static volatile uint8_t  CmdQuietVM  = 0;
static volatile uint16_t CmdQuietCnt = 0;
static volatile uint16_t CmdFrameIdleMs = 0;
static volatile uint16_t cCmd        = 0;
static volatile uint8_t  cbitCnt     = 0;
static volatile uint8_t  nextBitLen  = CBITLEN;
static volatile uint32_t CmdSuppressUntil = 0;

/* ====== プロトタイプ ====== */
static void QA_Init(void);
static void QA_1msTick(void);
static void QA_Task(void);
static inline void QA_ResetCmdFrame(void);
static void QA_SuppressRx(uint32_t ms);
static void QA_HandleCmd(int cmd);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//=========================================================================
// COMMAND Helper for DEBUG
//=========================================================================

#define CMD_VOL_UP		9
#define CMD_VOL_DOWN	10
#define CMD_MUTE		4

/* Quantum受信: 音量・ミュートのみ実行 */
#define QA_CMD_MUTE      0x018u
#define QA_CMD_MUTE_STAR 0x01bu
#define QA_CMD_VOL_UP    0x041u
#define QA_CMD_VOL_DOWN  0x044u
#define CMD_DEBOUNCE_MS  200u
#define MUTE_DEBOUNCE_MS 800u
#define VOLUP_GUARD_MS   250u  /* VolUp押下後の044(解放)をVolDownと誤認しない */
#define CMD_FRAME_IDLE_MS 100u  /* ビット途中で途切れたフレームを破棄 */
#define CMD_RX_SUPPRESS_MS 800u /* ユーザー操作後の誤受信抑止 */
#define CMD_RX_SUPPRESS_BAD_MS 1000u /* 非ユーザーcmd受信後の抑止 */

#ifdef QA_RX_TRACE
static void QA_Trace(const char *tag, uint16_t cmd)
{
  printf("[%s %03X m=%u v=%d]\n", tag, (unsigned)cmd,
         (unsigned)((IsMuted != 0u) ? 1u : 0u), cVolume);
}
#else
static void QA_Trace(const char *tag, uint16_t cmd)
{
  (void)tag;
  (void)cmd;
}
#endif

/* Quantum: 押下=code, 解放=code+3。VolUp解放(044)はVolDownと同コードのためペア化 */
static int QA_PopFiltered(void)
{
  int cmd;
  uint16_t ucmd;
  int next;

  for (;;) {
    cmd = cmdq_pop();
    if (cmd < 0) {
      return -1;
    }
    ucmd = (uint16_t)cmd;

    if (ucmd == QA_CMD_MUTE_STAR) {
      QA_Trace("skip", ucmd);
      continue;
    }

    next = cmdq_peek();
    if (ucmd == QA_CMD_VOL_UP && next == (int)QA_CMD_VOL_DOWN) {
      (void)cmdq_pop();
      QA_Trace("pair", ucmd);
    }

    if (ucmd == QA_CMD_VOL_UP || ucmd == QA_CMD_VOL_DOWN || ucmd == QA_CMD_MUTE) {
      while (!cmdq_is_empty() && (uint16_t)cmdq_peek() == (int)ucmd) {
        (void)cmdq_pop();
      }
    }

    QA_Trace("pop", ucmd);
    return cmd;
  }
}

static bool QA_IsUserCmd(uint16_t cmd)
{
  switch (cmd) {
  case QA_CMD_MUTE:
  case QA_CMD_VOL_UP:
  case QA_CMD_VOL_DOWN:
    return true;
  default:
    return false;
  }
}

const char *LastCmd=NULL;
int LastCmdNo=-1;

#ifdef CMD_DEBUG
const char *cmdName[42]={
  //00-GROUPE
  "9","12","6","StatusReport","MUTE","MUTE*","3","ReLeaseBrakes","ApplyBrakes",
  //01-GROUPE
	"VolumeUp",	"VolumeDwon","15","15*","4","4*","5","5*",
	"10","10*","11","11*","7","7*","2","2*",
	"8","8*","13","13*","14","14*","1","1*",
  //10-GROUPE
  "STC","RTC","DisconnectStandby","StartUp","StartUp*","Shutdown","Shutdown*",
  //Addtional GROUPE(Whistle OFFは非表示)
  "Whistle","BELL"
};

const uint16_t cmdCode[42]={
  //00-GROUPE
  0x000,0x005,0x009,0x014,0x018,0x01b,0x02d,0x035,0x30,
  //01-GROUPE
  0x041,0x044,0x048,0x04b,0x04d,0x04e,0x050,0x053,0x055,
  0x056,0x059,0x05a,0x060,0x063,0x069,0x06a,0x06c,0x06f,
  0x074,0x077,0x071,0x072,0x078,0x07b,
  //10-GROUPE
  0x09a,0x099,0x0a0,0x0a5,0x0a7,0x0a9,0x0aa,
  //Addtional GROUPE(Whistle OFFは非表示)
  0xF001,0xF002
};

int GetCmdNo(uint16_t cmd){
  int i;
  for(i=0;i<42;i++){
    if(cmdCode[i]==cmd){
      return i;
    }
  }
  return -1;
}
#endif

/* 1msごとに呼ばれるコールバック */
#define RX_BUF_SIZE 128
static uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

volatile unsigned int MS_COUNT=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM14)
  {
	  QA_1msTick();
	  if(MS_COUNT>0){
		  MS_COUNT--;   // 任意の処理（フラグやソフトタイマ更新など）
	  }
  }
}

void delay_ms(unsigned int ms){
	MS_COUNT=ms;
	while(MS_COUNT);
}

//====================================================================
// ADC関連
//====================================================================


int AdcRead(){
	// Errorなら -1
	int val=-1;
    HAL_ADC_Start(&hadc1);
    if(HAL_ADC_PollForConversion(&hadc1, 2)==HAL_OK){
    	val = (int)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    return val;
}
uint32_t adc_tick;
int PowerMV=0;	//PowerPackの電圧（mV単位）

static inline uint32_t Elapsed(uint32_t last)
{
    return HAL_GetTick() - last;
}

int32_t Get_mV(int adcval){
	return (int32_t)adcval*3300/4095;
}

//PowerPackの入力電圧をVで表示(unit=0.1V)
int GetPower_mV(int x){
	int v;
	v=Get_mV(x)*10;
	return v;
}

//====================================================================
// SOUND 関連
//====================================================================
// 公開API
HAL_StatusTypeDef SoundInit(void);                 // 初期化（BTL直結/Class-D前提）
// Forward declaration（先に宣言して暗黙宣言を防止）
static HAL_StatusTypeDef SOUND_Rd1(uint8_t cmd, uint8_t *out, uint32_t to_ms);

// ==== CS制御 ====
static inline void SOUND_CS_L(void){ HAL_GPIO_WritePin(NCS_GPIO_Port, NCS_Pin, GPIO_PIN_RESET); }
static inline void SOUND_CS_H(void){ HAL_GPIO_WritePin(NCS_GPIO_Port, NCS_Pin, GPIO_PIN_SET); }

// ==== 設定マクロ ====
// SPIタイムアウト
#ifndef SOUND_SPI_TIMEOUT_MS
#define SOUND_SPI_TIMEOUT_MS   50u
#endif
// 出力方式（BTL直結するのでClass-D）
#ifndef SOUND_USE_CLASS_D
#define SOUND_USE_CLASS_D      1   // 1:Class-D(BTL), 0:Class-AB(単端)
#endif

// ==== CBUSYB=H 待ち ====
static HAL_StatusTypeDef SOUND_WaitReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(CBUSY_GPIO_Port, CBUSY_Pin) != GPIO_PIN_SET) {
        if ((HAL_GetTick() - start) >= timeout_ms) return HAL_TIMEOUT;
    }
    return HAL_OK;
}

// ==== SPI送信（CBUSYB監視つき）====
static HAL_StatusTypeDef SOUND_Tx(const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    HAL_StatusTypeDef st = SOUND_WaitReady(timeout_ms);
    if (st != HAL_OK) return st;

    SOUND_CS_L();
    st = HAL_SPI_Transmit(&hspi1, (uint8_t*)buf, len, timeout_ms);
    SOUND_CS_H();
    return st;
}

// ==== コマンド低層 ====

// PUP（1B, WCM=0）: パワーダウン解除
static HAL_StatusTypeDef SOUND_SendPUP(uint32_t to_ms)
{
    const uint8_t b = 0x00; // 0000 0000b
    return SOUND_Tx(&b, 1, to_ms);
}

// AMODE（2B）: 0000 01 DAMP HPF / FAD DAG1 DAG0 AIG1 AIG0 AEN1 AEN0 POP
static HAL_StatusTypeDef SOUND_SendAMODE(
    uint8_t DAMP, uint8_t HPF,
    uint8_t FAD,  uint8_t DAG,      // 0..3
    uint8_t AIG,                     // 0..2
    uint8_t AEN1, uint8_t AEN0,
    uint8_t POP,
    uint32_t to_ms)
{
    uint8_t b1 = (uint8_t)(0x04 | ((DAMP & 1) << 1) | (HPF & 1));
    uint8_t b2 = (uint8_t)((FAD & 1) << 7 | (DAG & 3) << 5 | (AIG & 3) << 3 |
                           (AEN1 & 1) << 2 | (AEN0 & 1) << 1 | (POP & 1));
    const uint8_t tx[2] = { b1, b2 };
    return SOUND_Tx(tx, 2, to_ms);
}

// AVOL（2B）: 0x08, code（例: 0x24=0 dB）
static HAL_StatusTypeDef SOUND_SendAVOL(uint8_t av_code, uint32_t to_ms)
{
    const uint8_t tx[2] = { 0x08, av_code };
    return SOUND_Tx(tx, 2, to_ms);
}

// CVOL（2B）: 1010 CH3 CH2 CH1 CH0, 0 CV1 CV0 CV6 CV5 CV4 CV3 CV2
#if 0
static HAL_StatusTypeDef SOUND_SendCVOL(uint8_t ch, uint8_t vol7, uint32_t to_ms)
{
    if (ch > 3) return HAL_ERROR;
    const uint8_t b1 = (uint8_t)(0xA0 | (1u << (ch & 0x03)));
    const uint8_t b2 = (uint8_t)(vol7 & 0x7F);  // 7bit
    const uint8_t tx[2] = { b1, b2 };
    return SOUND_Tx(tx, 2, to_ms);
}
#else
static HAL_StatusTypeDef SOUND_SendCVOL(uint8_t ch, uint8_t vol7, uint32_t to_ms)
{
    if (ch > 3) return HAL_ERROR;

    const uint8_t b1 = (uint8_t)(0xA0 | (1u << (ch & 0x03)));

    vol7 &= 0x7F;

    // 2バイト目: 0 CV1 CV0 CV6 CV5 CV4 CV3 CV2
    const uint8_t b2 =
        (uint8_t)(
            ((vol7 & 0x03u) << 5) |   // CV1,CV0 -> bit6,5
            ((vol7 & 0x7Cu) >> 2)     // CV6..CV2 -> bit4..0
        );

    const uint8_t tx[2] = { b1, b2 };
    return SOUND_Tx(tx, 2, to_ms);
}
#endif
// PLAY（2B）: [0100 F9 F8 C1 C0], [F7..F0]（0..1023）
static HAL_StatusTypeDef SOUND_SendPLAY(uint8_t ch, uint16_t phrase, uint32_t to_ms)
{
    if (ch > 3) return HAL_ERROR;
    phrase &= 0x03FF; // F9..F0

    uint8_t b1 = 0x40;                              // 0100 xxxx
    b1 |= (uint8_t)(((phrase >> 8) & 0x03) << 2);   // F9,F8 → bit3:2
    b1 |= (uint8_t)(ch & 0x03);                     // C1,C0 → bit1:0
    const uint8_t b2 = (uint8_t)(phrase & 0xFF);

    const uint8_t tx[2] = { b1, b2 };
    return SOUND_Tx(tx, 2, to_ms);
}

// STOP（1B）: [0110 CH3 CH2 CH1 CH0]
static HAL_StatusTypeDef SOUND_SendSTOP(uint8_t ch, uint32_t to_ms)
{
    if (ch > 3) return HAL_ERROR;
    const uint8_t b1 = (uint8_t)(0x60 | (1u << (ch & 0x03)));
    return SOUND_Tx(&b1, 1, to_ms);
}

// ==== 公開API ====
void     StopAll();

// 初期化：PUP → AMODE(Class-D/BTL) → AVOL(0dB) → CVOL(全CH=0dB)
HAL_StatusTypeDef SoundInit(void)
{
    HAL_StatusTypeDef st;
    HAL_Delay(10); // 立ち上がり余裕

    // 1) PUP
    st = SOUND_SendPUP(SOUND_SPI_TIMEOUT_MS);
    if (st != HAL_OK) return st;

    StopAll();

    // 2) AMODE
#if (SOUND_USE_CLASS_D)
    // Class-D（BTL直結）: DAMP=1, HPF=0, FAD=0, DAG=0, AIG=0, AEN=01, POP=0
    st = SOUND_SendAMODE(1, 0, 0, 0, 0, 0, 1, 0, SOUND_SPI_TIMEOUT_MS);
#else
    // Class-AB（単端/コンデンサ直列）: DAMP=0, AEN=01
    st = SOUND_SendAMODE(0, 0, 0, 0, 0, 0, 1, 0, SOUND_SPI_TIMEOUT_MS);
#endif
    if (st != HAL_OK) return st;

    // 3) AVOL: 0 dB（例: 0x24）
    st = SOUND_SendAVOL(0x24, SOUND_SPI_TIMEOUT_MS);
    if (st != HAL_OK) return st;

    // 4) CVOL: 全CH 0 dB（=0x00を各CHに設定でも良い）
    for (uint8_t ch = 0; ch < 4; ch++) {
        st = SOUND_SendCVOL(ch, 0x00, SOUND_SPI_TIMEOUT_MS);
        if (st != HAL_OK) return st;
    }
    return HAL_OK;
}

HAL_StatusTypeDef Sound_ReadVersion(uint8_t *ver) {     // RDVER : 0xB4
    return SOUND_Rd1(0xB4, ver, SOUND_SPI_TIMEOUT_MS);
}
HAL_StatusTypeDef Sound_ReadError(uint8_t *err) {       // RDERR : 0xB8
    return SOUND_Rd1(0xB8, err, SOUND_SPI_TIMEOUT_MS);
}

// ================== 4ch/音量0-255の高レベルAPI ==================
//
// 機能:
// ・4チャネル同時再生
// ・全体/個別ボリュームを0〜255（0=最小/ミュート, 255=最大）で指定
// ・空きチャネル自動割り当て対応
//
// 既存の SOUND_SendPLAY / STOP / CVOL / AVOL を内部で使用

static inline int in_range_int(int x, int lo, int hi){
    return (unsigned)(x - lo) <= (unsigned)(hi - lo);
}

// ---- RDSTAT: BUSYB/NCR 用 ----
static HAL_StatusTypeDef SOUND_Rd1(uint8_t cmd, uint8_t *out, uint32_t to_ms)
{
    HAL_StatusTypeDef st;
    uint8_t dummy = 0x00;
    st = SOUND_WaitReady(to_ms); if (st!=HAL_OK) return st;

    SOUND_CS_L(); st = HAL_SPI_Transmit(&hspi1, &cmd, 1, to_ms); SOUND_CS_H();
    if (st!=HAL_OK) return st;

    SOUND_CS_L(); st = HAL_SPI_TransmitReceive(&hspi1, &dummy, out, 1, to_ms); SOUND_CS_H();
    return st;
}

static HAL_StatusTypeDef Sound_ReadStatus(uint8_t *status)
{
    return SOUND_Rd1(0xB0, status, SOUND_SPI_TIMEOUT_MS);
}
#define RDSTAT_BUSYB(st,ch) (((st) >> (4 + ((ch)&3))) & 1)  // 1=アイドル, 0=再生中

// ---------------- 再生API ----------------

// 明示チャネル指定で再生
int PlayOn(int ch, int phrase)
{
    if (!in_range_int(ch,0,3) || !in_range_int(phrase,0,1023)) return -1;
    return (SOUND_SendPLAY((uint8_t)ch, (uint16_t)phrase, 50) == HAL_OK) ? ch : -1;
}

// 空きチャネルへ自動割り当て
int PlayAuto(int phrase)
{
    if (!in_range_int(phrase,0,1023)) return -1;
    uint8_t st=0;
    if (Sound_ReadStatus(&st) != HAL_OK) return -1;

    for (int ch=0; ch<4; ch++){
        if (RDSTAT_BUSYB(st,ch)) {
            if (SOUND_SendPLAY((uint8_t)ch, (uint16_t)phrase, 50) == HAL_OK) return ch;
        }
    }
    return -1; // 全ch再生中
}

// 停止系
void StopOn(int ch)  { if (in_range_int(ch,0,3)) (void)SOUND_SendSTOP((uint8_t)ch, 50); }
void StopAll(void)   { for (uint8_t ch=0; ch<4; ch++) (void)SOUND_SendSTOP(ch, 50); }

// ---------------- 音量変換 ----------------
//
// CVOL: 0=最大, 0x7F=ミュート
// AVOL: 0〜15段, 値が大きいほど減衰
// ユーザー入力は共通で 0〜255（0=最小/ミュート, 255=最大）

static inline uint8_t map255_to_cvol(uint8_t ui)
{
	uint16_t t = (uint16_t)ui * 0x7Fu + 127u/2u;
    return (uint8_t)(0x7F - (t / 255u));
}

static const uint8_t s_avol_code_tbl[16] = {
    0x36,0x34,0x32,0x30,0x2E,0x2C,0x2A,0x28,
    0x26,0x25,0x24,0x23,0x22,0x21,0x20,0x1E
};
static inline uint8_t map255_to_avol_code(uint8_t ui)
{
    uint16_t t = (uint16_t)ui * 15u + 127u;
    uint8_t idx = (uint8_t)(15u - (t / 255u));
    return s_avol_code_tbl[idx];
}

// 音量設定
void SetCVol(int ch, uint8_t vol0_255)
{
    if (!in_range_int(ch,0,3)) return;
    uint8_t cv = map255_to_cvol(vol0_255);
    (void)SOUND_SendCVOL((uint8_t)ch, cv, 50);
}
void SetCVolAll(uint8_t vol0_255)
{
    uint8_t cv = map255_to_cvol(vol0_255);
    for (uint8_t ch=0; ch<4; ch++) (void)SOUND_SendCVOL(ch, cv, 50);
}

void _SetCVolAll(uint8_t cv)
{
    /* CVOL: 0=最大, 127=ミュート。振幅相当 amp=(127-cv)
     *
     * 重ね時に走行音量を変えない方針:
     *  - 常に各CHの上限を master の約1/2(-6dB)に固定（MIX_CH_SCALE）
     *  - 走行は常にその上限（重ね中も一定）
     *  - ラジエターは 0→上限→0 でフェード
     *  - 両者最大時の合計 ≈ master フル → 合成クランプしない
     * 結果として単音時も約-6dB下がるが、重ねで音量が変わらない。 */
#define MIX_CH_SCALE  128u  /* /255 ≈ 0.5 (-6dB)。まだクリップするなら 100〜110 に下げる */
    uint16_t master_amp = (uint16_t)(127u - cv);
    uint16_t ch_ceil = (master_amp * MIX_CH_SCALE) / 255u;
    uint16_t run_amp = ch_ceil;
    uint16_t fan_amp = (ch_ceil * (uint16_t)FanVolScale) / 255u;
    uint8_t run_cv = (uint8_t)(127u - run_amp);
    uint8_t fan_cv = (uint8_t)(127u - fan_amp);

    (void)SOUND_SendCVOL(CH_RUN, run_cv, 50);
    (void)SOUND_SendCVOL(CH_FAN, fan_cv, 50);
    (void)SOUND_SendCVOL(2, 127, 50);
    (void)SOUND_SendCVOL(3, 127, 50);
#undef MIX_CH_SCALE
}


void SetAVol(uint8_t vol0_255)
{
    uint8_t code = map255_to_avol_code(vol0_255);
    (void)SOUND_SendAVOL(code, 50);
}

// ---------------- 下位互換ラッパ ----------------
HAL_StatusTypeDef Play(int ch)
{
    if (!in_range_int(ch,0,3)) return HAL_ERROR;
    return (PlayOn(ch, ch) >= 0) ? HAL_OK : HAL_ERROR;
}
HAL_StatusTypeDef Stop(int ch)
{
    if (!in_range_int(ch,0,3)) return HAL_ERROR;
    StopOn(ch); return HAL_OK;
}
HAL_StatusTypeDef SetVolume(int ch, uint8_t vol7)
{
    if (!in_range_int(ch,0,3)) return HAL_ERROR;
    uint8_t vol255 = (uint8_t)(255u - ((uint16_t)vol7 * 255u / 0x7Fu));
    SetCVol(ch, vol255);
    return HAL_OK;
}

//====================================================================
// printfサポート
//====================================================================
/* UART送信（ブロッキング）によるprintf対応
 * DMA版は連続printf時にHAL/DMA割込と競合しHardFaultの原因になるため不使用 */
int _write(int file, char *ptr, int len)
{
    uint16_t tx_len;

    (void)file;
    if (len <= 0) {
        return 0;
    }
    tx_len = (len > 65535) ? 65535u : (uint16_t)len;
    (void)HAL_UART_Transmit(&huart1, (const uint8_t *)ptr, tx_len, HAL_MAX_DELAY);
    return len;
}

/* UART受信割り込みによるscanf対応 */
int _read(int file, char *ptr, int len)
{
    int count = 0;
    while (count < len)
    {
        // データが来るまで待機
        while (rx_head == rx_tail) { /* wait */ }

        char c = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % RX_BUF_SIZE;

        // エコーバック（送信）
        HAL_UART_Transmit(&huart2, (uint8_t*)&c, 1, HAL_MAX_DELAY);

        ptr[count++] = c;

        // 改行入力時に終了（scanfだとバッファに残るが）
        if (c == '\n' || c == '\r') break;
    }
    return count;
}

/* 受信割り込み開始 */
static uint8_t uart2_rx_hcar;

void UART2_Start_Receive_IT(void)
{
    HAL_UART_Receive_IT(&huart2, &uart2_rx_hcar, 1);
}

/* 割り込みコールバック（受信完了時） */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uint8_t c = huart->pRxBuffPtr[-1]; // 直前に受信したデータ
        uint16_t next_head = (rx_head + 1) % RX_BUF_SIZE;
        if (next_head != rx_tail) // バッファあふれ防止
        {
            rx_buf[rx_head] = c;
            rx_head = next_head;
        }
        // 次の1文字受信開始
        UART2_Start_Receive_IT();
    }
}

static inline void QA_ResetCmdFrame(void)
{
  cCmd = 0;
  cbitCnt = 0;
  CmdMode = false;
  CmdFrameArmed = false;
  CmdFrameIdleMs = 0;
  nextBitLen = CBITLEN;
}

static inline bool QA_RxAllowed(void)
{
  return (int32_t)(HAL_GetTick() - CmdSuppressUntil) >= 0;
}

static void QA_SuppressRx(uint32_t ms)
{
  int dropped = 0;

  CmdSuppressUntil = HAL_GetTick() + ms;
  QA_ResetCmdFrame();
  while (cmdq_pop() >= 0) {
    dropped++;
  }
#ifdef QA_RX_TRACE
  if (dropped > 0) {
    printf("[drop %d]\n", dropped);
  }
#else
  (void)dropped;
#endif
}

static void QA_1msTick(void)
{
  // TIMER_INTERVAL ms毎にここが呼び出される
  int cVM;
  int cState;
  bool validBit;
  bool validBell;

  // Noise Cancel =====================
  cVM = digitalRead_VM(); // VM_PIN → VMIN
  if (cVM == lVM) { VM = (uint8_t)cVM; }
  lVM = (uint8_t)cVM;
  // ==================================
  // コマンド待受: VMがPW_LONG超えて安定したらフレーム受付可能
  if (VM == CmdQuietVM) {
    if (CmdQuietCnt < 0xFFFFu) {
      CmdQuietCnt++;
    }
    if (CmdQuietCnt > PW_LONG) {
      if (!CmdMode && QA_RxAllowed()) {
        CmdFrameArmed = true;
      } else if (cbitCnt == 0) {
        /* 受信開始のみでビット未確定のまま静止 → 誤開始を解除 */
        CmdMode = false;
        CmdFrameArmed = true;
      }
    }
  } else {
    CmdQuietVM = VM;
    CmdQuietCnt = 0;
  }
  if (CmdMode && cbitCnt > 0) {
    if (CmdFrameIdleMs < 0xFFFFu) {
      CmdFrameIdleMs++;
    }
    if (CmdFrameIdleMs > CMD_FRAME_IDLE_MS) {
      QA_ResetCmdFrame();
    }
  } else {
    CmdFrameIdleMs = 0;
  }
  /* 尾灯方向の確定はメインループ（電源復帰＋短デバウンス）で行う。
   * ここでは Quantum 用の VM 安定計測のみ（DirHoldMs は互換のため維持）。 */
  if (DirPowerOk == 0U) {
    DirLastVM = (uint8_t)cVM;
    DirHoldMs = 0;
  } else if (cVM == (int)DirLastVM) {
    if (DirHoldMs < 0xFFFFu) {
      DirHoldMs++;
    }
  } else {
    DirLastVM = (uint8_t)cVM;
    DirHoldMs = 0;
  }

  // VM から状態判定
  if(VM){
	    cState = QS_RESET;
  }else{
	    cState = QS_SET;
  }
  if (cState == QS_IDLE) {
    // 何もしない
    LastState = cState;
    return;
  }
  digitalWrite_IO2(cState-1);

  if (cState != LastState) {
    CmdFrameIdleMs = 0;
    /* 極性エッジ直後はDIRのVM追従を禁止（汽笛・コマンド開始） */
    DirQaHoldUntil = HAL_GetTick() + (uint32_t)DIR_QA_HOLD_MS;
    validBit = (PulseWidth >= PW_MIN) && (PulseWidth <= PW_LONG);
    validBell = (PulseWidth >= PW_BELL_MIN) && (PulseWidth <= PW_BELL_MAX);

    // 汽笛が鳴っていれば停止（キューには載せない）
    if (IsWhistle) {
      IsWhistle = false;
      QA_ResetCmdFrame();
    }

    if (!QA_RxAllowed()) {
      PulseWidth = 0;
      LastState  = cState;
      return;
    }

    if (CmdMode || CmdFrameArmed) {
      if (!CmdMode) {
        /* 待受中の先頭エッジ: 旧Arduino同様CmdModeのみ開始（PulseWidthは次エッジで有効） */
        CmdMode = true;
        CmdFrameArmed = false;
      } else if (validBit) {
        int bit = (PulseWidth < PW_CHK) ? 0 : 1;
        digitalWrite_IO2(bit);
        cCmd = (uint16_t)((cCmd << 1) + (uint16_t)bit);
        cbitCnt++;
        if (cbitCnt >= nextBitLen) {
          uint16_t rxCmd = cCmd;
          /* MUTE*(01b)はボタン解放パルス。キューに載せない */
          if (rxCmd == QA_CMD_MUTE_STAR) {
            cbitCnt = 0;
            cCmd = 0;
            nextBitLen = CBITLEN;
            CmdMode = false;
            CmdFrameArmed = false;
            PulseWidth = 0;
            LastState  = cState;
            return;
          }
          if (QA_IsUserCmd(rxCmd)) {
            cmdq_push_isr((int)rxCmd);
          } else {
            CmdSuppressUntil = HAL_GetTick() + CMD_RX_SUPPRESS_BAD_MS;
            QA_ResetCmdFrame();
            PulseWidth = 0;
            LastState  = cState;
            return;
          }
          cbitCnt = 0;
          cCmd = 0;
          nextBitLen = CBITLEN;
          CmdMode = false;
          CmdFrameArmed = false;
        }
      } else if (validBell) {
        /* ベル等ユーザー未使用コマンドは無視 */
        CmdSuppressUntil = HAL_GetTick() + CMD_RX_SUPPRESS_BAD_MS;
        QA_ResetCmdFrame();
      } else if (cbitCnt > 0) {
        /* ビット途中で幅不正 → フレーム破棄 */
        QA_ResetCmdFrame();
      }
    }

    // 反転からカウント再開
    PulseWidth = 0;
    LastState  = cState;
    return;
  }

  // 状態継続中はカウントアップ＋可視化
  digitalWrite_IO0(IsWhistle); // 旧: digitalWrite(4, IsWhistle)
  //digitalWrite_IO1(CmdMode);   // 旧: digitalWrite(5, CmdMode)

  if (LastState == cState) {
    if (PulseWidth <= PW_WHISTLE) {
      PulseWidth++;
    } else {
      // 汽笛ローカル表示のみ（キューには載せない）
      if (CmdMode && (!IsWhistle)) {
        IsWhistle = true;
      }
    }
    /* 汽笛・コマンド中は追従禁止を延長 */
    if (CmdMode || IsWhistle) {
      DirQaHoldUntil = HAL_GetTick() + (uint32_t)DIR_QA_HOLD_MS;
    }
    LastState = cState;
  }
}

/* ====== メインループ側：キューを吐き出して処理 ====== */
static void QA_HandleCmd(int cmd)
{
  static uint32_t lastActTick = 0;
  static uint16_t lastActCmd = 0;
  static uint32_t lastMuteTick = 0;
  uint32_t now = HAL_GetTick();
  uint16_t ucmd = (uint16_t)cmd;
  uint8_t wasMuted;

  /* 破損値(24等)を 0/1 に正規化。非0はミュート中 */
  IsMuted = (IsMuted != 0u) ? 1u : 0u;

  switch (ucmd) {
  case QA_CMD_VOL_UP:
  case QA_CMD_VOL_DOWN:
    if (lastActCmd == ucmd && (now - lastActTick) < CMD_DEBOUNCE_MS) {
      QA_Trace("deb", ucmd);
      return;
    }
    lastActCmd = ucmd;
    lastActTick = now;
    break;
  case QA_CMD_MUTE:
    if ((now - lastMuteTick) < MUTE_DEBOUNCE_MS) {
      QA_Trace("deb", ucmd);
      return;
    }
    lastMuteTick = now;
    break;
  default:
    return;
  }

  switch (ucmd) {
  case QA_CMD_VOL_UP:
    VolumeUp();
    VolUpGuardUntil = now + VOLUP_GUARD_MS;
#ifdef CMD_DEBUG
    printf("Volume Up\n");
#endif
    QA_SuppressRx(CMD_RX_SUPPRESS_MS);
    break;
  case QA_CMD_VOL_DOWN:
    if ((int32_t)(now - VolUpGuardUntil) < 0) {
      QA_Trace("rel", ucmd);
      QA_SuppressRx(CMD_RX_SUPPRESS_MS);
      break;
    }
    VolumeDown();
#ifdef CMD_DEBUG
    printf("Volume Down\n");
#endif
    QA_SuppressRx(CMD_RX_SUPPRESS_MS);
    break;
  case QA_CMD_MUTE:
    wasMuted = IsMuted;
    if (wasMuted) {
      SetMute(0);
#ifdef CMD_DEBUG
      printf("Mute OFF\n");
#endif
    } else {
      SetMute(1);
#ifdef CMD_DEBUG
      printf("Mute ON\n");
#endif
    }
    QA_SuppressRx(CMD_RX_SUPPRESS_MS);
    break;
  default:
    break;
  }
}

static void QA_Task(void)
{
  int cmd;

  cmd = QA_PopFiltered();
  if (cmd < 0) {
    return;
  }
#if 0 /* CMD_DEBUG: コマンド名・番号表示 */
  int cmdNo = GetCmdNo((uint16_t)cmd);
  if (cmdNo >= 0) {
    printf("%s(%04X)\n", cmdName[cmdNo], (unsigned)cmd);
    LastCmd = cmdName[cmdNo];
    LastCmdNo = cmdNo;
  } else {
    printf("CMD: 0x%04X\n", (unsigned)cmd);
    LastCmd = NULL;
    LastCmdNo = -1;
  }
#endif
  QA_HandleCmd(cmd);
}

static void QA_Init(void)
{
  // 出力初期化（IO0/IO1 は CubeMX 側で Output 設定済み前提）
  digitalWrite_IO0(0);
  //digitalWrite_IO1(0);
  // 状態初期化
  lVM = digitalRead_VM();
  VM = lVM;
  DirLastVM = lVM;
  DirHoldMs = 0;
  DirPowerOffMs = 0;
  DirRelatchArmed = 0U;
  DirPowerOk = 1U;
  DirRecoverMs = 0;
  DirPowerWasOff = 0U;
  DirPowerPeakMv = 0;
  DirArmLogged = 0U;
  DirAllowArm = 0U;
  DirHadSolidOn = 0U;
  DirPostChangeLock = 0U;
  DirOnSettleMs = 0;
  DirVmPowerGood = 0U;
  DirVmCand = 0xFFu;
  DirVmStableMs = 0;
  DirInitPending = 1U;
  DirInitCand = 0xFFu;
  DirInitStableMs = 0;
  DirFollowCand = 0xFFu;
  DirFollowStableMs = 0;
  DirQaHoldUntil = 0;
  DirVinWinMax = 0;
  DirVinWinMin = 0xFFFFu;
  DirVinWinMs = 0;
  DirVinSwingOk = 0U;
  DirVinSwingUntil = 0;
#ifdef POWER_VOLT_LOG
  DirLogLastVm = -1;
  DirLogVmTick = HAL_GetTick();
#endif
  IsNormalDir = true; /* PowerONでVMから確定するまで仮の正転 */
  LastLoggedDir = IsNormalDir; /* 起動直後の偽 DIR_NOW を出さない */
  SetTailLED(IsNormalDir);
  /* QS_IDLE のままだと 1ms ティック最初の「擬似エッジ」(QS_IDLE→SET/RESET) で CmdMode が true になり、
   * 状態が安定しているだけで約 PW_WHISTLE ms 後に汽笛(ホイッスル)が誤発火する。
   * 現在の VM と同じ QS を LastState に入れておけば、実際の極性変化までエッジにならない。 */
  LastState = VM ? QS_RESET : QS_SET;
  PulseWidth = 0;
  QA_ResetCmdFrame();
  IsWhistle  = false;
  CmdQuietVM = VM;
  CmdQuietCnt = 0;

  // TIM14 1ms 割り込み開始（CubeMXで 1kHz 設定済みを想定）
  HAL_TIM_Base_Start_IT(&htim14);
}

#if 0
void SPI1_SanityTest(void) {
    uint8_t pat[2] = {0xAA, 0x55};
    for (int i=0; i<4; i++) {
        HAL_GPIO_WritePin(NCS_GPIO_Port, NCS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, pat, sizeof pat, 100);
        HAL_GPIO_WritePin(NCS_GPIO_Port, NCS_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
    }
}
#endif


//=============================================================================
// Play Test
//=============================================================================
// ──────────────────────────────────────────────────────────────
// ツールのCSV手順をほぼそのまま再現する再生シーケンス
//   送信順： PUP → AMODE(0x04,0xC3) → 0x0C,0x01 → AVOL(0x08,0x24)
//            CVOL A1/A2/A4/A8 = 0dB → 0x30,0x00 → RDSTAT → 0x51(開始)
//            以降 RDSTAT を約2ms周期でポーリング
//   注: CSVのサブms間隔は HAL_Delay では表現できないので、1ms未満は無待ち。
// ──────────────────────────────────────────────────────────────

static HAL_StatusTypeDef tx1(uint8_t b, uint32_t to_ms)
{
    return SOUND_Tx(&b, 1, to_ms);
}

static HAL_StatusTypeDef tx2(uint8_t b1, uint8_t b2, uint32_t to_ms)
{
    uint8_t tx[2] = { b1, b2 };
    return SOUND_Tx(tx, 2, to_ms);
}

// RDSTATをツール流儀（ダミー=0xFF）で1バイト読む
static HAL_StatusTypeDef rdstat_ff(uint8_t *p, uint32_t to_ms)
{
    HAL_StatusTypeDef st;
    uint8_t cmd = 0xB0;
    uint8_t dummy = 0xFF;

    // コマンド（CS↓で1B）
    SOUND_CS_L();
    st = HAL_SPI_Transmit(&hspi1, &cmd, 1, to_ms);
    SOUND_CS_H();
    if (st != HAL_OK) return st;

    // ダミーを送りながら1B受信（CS↓で1B）
    SOUND_CS_L();
    st = HAL_SPI_TransmitReceive(&hspi1, &dummy, p, 1, to_ms);
    SOUND_CS_H();

    return st;
}

// CSVと同順・同感覚の「開始」シーケンス
HAL_StatusTypeDef Sound_PlayLikeTool(void)
{
    HAL_StatusTypeDef st;
    uint8_t s;

    // 1) PUP (0x00)
    st = tx1(0x00, 50);                  if (st != HAL_OK) return st;
    HAL_Delay(7);                        // ≈ 7.014 ms

    // 2) AMODE 相当（0x04, 0xC3）
    st = tx2(0x04, 0xC3, 50);            if (st != HAL_OK) return st;
    HAL_Delay(39);                       // ≈ 38.557 ms

    // 3) 不明コマンド（CSV: 0x0C, 0x01）
    st = tx2(0x0C, 0x01, 50);            if (st != HAL_OK) return st;
    // 以下の数百µs級はHAL_Delayでは無視

    // 4) AVOL 0 dB（0x08, 0x24）
    st = tx2(0x08, 0x24, 50);            if (st != HAL_OK) return st;

    // 5) CVOL 各CH = 0 dB（A1/A2/A4/A8, 00）
    st = tx2(0xA1, 0x00, 50);            if (st != HAL_OK) return st;
    st = tx2(0xA2, 0x00, 50);            if (st != HAL_OK) return st;
    st = tx2(0xA4, 0x00, 50);            if (st != HAL_OK) return st;
    st = tx2(0xA8, 0x00, 50);            if (st != HAL_OK) return st;

    // 6) CSV: 0x30, 0x00（ツール側の動作モード設定の一種）
    st = tx2(0x30, 0x00, 50);            if (st != HAL_OK) return st;

    // 7) RDSTAT（1回目）：B0 → (ダミーFFで1B受信)
    st = rdstat_ff(&s, 50);              if (st != HAL_OK) return st;

    // 8) CSV: 0x51（1バイトの開始トリガ）
    st = tx1(0x51, 50);                  if (st != HAL_OK) return st;

    // 9) RDSTATポーリング（約2ms周期、≒CSV平均1.87ms）
    for (int i = 0; i < 2400; i++) {     // ≈ 4.8 秒ぶん
        st = rdstat_ff(&s, 50);
        if (st != HAL_OK) return st;
        HAL_Delay(2);
    }
    return HAL_OK;
}

// ==== ループ制御コマンド ====
// SLOOP（1B）: 1000 CH3 CH2 CH1 CH0
// CLOOP（1B）: 1001 CH3 CH2 CH1 CH0
// ch = 0..3
// 例: ch0 -> SLOOP=0x81, CLOOP=0x91

static HAL_StatusTypeDef SOUND_SendSLOOP(uint8_t ch)
{
    if (ch > 3) return HAL_ERROR;
    uint8_t b = (uint8_t)(0x80u | (1u << ch));
    return SOUND_Tx(&b, 1, SOUND_SPI_TIMEOUT_MS);
}

static HAL_StatusTypeDef SOUND_SendCLOOP(uint8_t ch)
{
    if (ch > 3) return HAL_ERROR;
    uint8_t b = (uint8_t)(0x90u | (1u << ch));
    return SOUND_Tx(&b, 1, SOUND_SPI_TIMEOUT_MS);
}

// ==== 公開API ====
// 通常再生
HAL_StatusTypeDef StdPlayOn(int ch, int phrase)
{
    if (!in_range_int(ch, 0, 3) || !in_range_int(phrase, 0, 1023)) {
        return HAL_ERROR;
    }

    return SOUND_SendPLAY((uint8_t)ch, (uint16_t)phrase, SOUND_SPI_TIMEOUT_MS);
}

// ループ再生
HAL_StatusTypeDef LoopOn(int ch, int phrase)
{
    HAL_StatusTypeDef st;

    if (!in_range_int(ch, 0, 3) || !in_range_int(phrase, 0, 1023)) {
        return HAL_ERROR;
    }

    st = SOUND_SendPLAY((uint8_t)ch, (uint16_t)phrase, SOUND_SPI_TIMEOUT_MS);
    if (st != HAL_OK) return st;

    // まずは短めで試す
    HAL_Delay(1);

    return SOUND_SendSLOOP((uint8_t)ch);
}

// ループ解除
HAL_StatusTypeDef LoopOff(int ch)
{
    if (!in_range_int(ch, 0, 3)) {
        return HAL_ERROR;
    }

    return SOUND_SendCLOOP((uint8_t)ch);
}


int IsPlaying(int ch)
{
    uint8_t st = 0;

    if (!in_range_int(ch, 0, 3)) return 0;
    if (Sound_ReadStatus(&st) != HAL_OK) return 0;

    // RDSTAT_BUSYB: 1=アイドル, 0=再生中
    return RDSTAT_BUSYB(st, ch) ? 0 : 1;
}

#ifdef SOUND_DEBUG
static const char *SoundPhraseName(int phrase)
{
  switch (phrase) {
  case PHRASE_FAN_LOOP: return "RadiatorFan";
  case PHRASE_RUN_LOOP: return "Run";
  case PHRASE_STARTUP:  return "Startup";
  default:              return "?";
  }
}

static void SoundDbg_Play(int phrase)
{
  printf("Sound PLAY P%d (%s)\n", phrase, SoundPhraseName(phrase));
}

static void SoundDbg_Loop(int phrase)
{
  printf("Sound LOOP P%d (%s)\n", phrase, SoundPhraseName(phrase));
}

static void SoundDbg_Stop(const char *reason)
{
  printf("Sound STOP (%s)\n", reason);
}
#else
static void SoundDbg_Play(int phrase)
{
  (void)phrase;
}

static void SoundDbg_Loop(int phrase)
{
  (void)phrase;
}

static void SoundDbg_Stop(const char *reason)
{
  (void)reason;
}
#endif

/* ラジエター重ね: 先頭2秒フェードイン、末尾2秒フェードアウト → 0..255 */
static uint8_t RunSound_FanFadeScale(uint32_t elapsed_ms)
{
  uint32_t remain;

  if (elapsed_ms >= RUN_FAN_DURATION_MS) {
    return 0U;
  }
  if (elapsed_ms < RUN_FAN_FADE_MS) {
    return (uint8_t)((elapsed_ms * 255u) / RUN_FAN_FADE_MS);
  }
  if (elapsed_ms >= (RUN_FAN_DURATION_MS - RUN_FAN_FADE_MS)) {
    remain = RUN_FAN_DURATION_MS - elapsed_ms;
    return (uint8_t)((remain * 255u) / RUN_FAN_FADE_MS);
  }
  return 255U;
}

static void RunSound_ApplyFanVol(uint8_t scale)
{
  if (FanVolScale == scale) {
    return;
  }
  FanVolScale = scale;
  _SetVolume();
}

#ifdef SOUND_DEBUG
static void RunSound_Countdown(uint32_t remain_ms, const char *phase)
{
  int sec;

  if (remain_ms == 0U) {
    sec = 0;
  } else {
    sec = (int)((remain_ms + 999u) / 1000u); /* 切り上げ秒 */
  }
  if (sec == FanCountdownLastSec) {
    return;
  }
  FanCountdownLastSec = sec;
  printf("FAN %s %ds (vol=%u/255)\n", phase, sec, (unsigned)FanVolScale);
}
#else
static void RunSound_Countdown(uint32_t remain_ms, const char *phase)
{
  (void)remain_ms;
  (void)phase;
}
#endif

static void RunSound_Reset(void)
{
  RunSoundState = runMainLoop;
  RunModeSince = 0;
  FanVolScale = 0U;
  FanCountdownLastSec = -1;
}

static void RunSound_StartMainLoop(void)
{
  FanVolScale = 0U;
  _SetVolume();
  (void)StopOn(CH_FAN);
  SoundDbg_Loop(PHRASE_RUN_LOOP);
  (void)LoopOn(CH_RUN, PHRASE_RUN_LOOP);
  RunSoundState = runMainLoop;
  RunModeSince = HAL_GetTick();
  FanCountdownLastSec = -1;
}

/* 軌道電源ON時: 音源を再初期化してから始動音（USB通電中の後付け電源でも可） */
static void SoundPowerOn_Start(void)
{
  (void)SoundInit();
  FanVolScale = 0U;
  _SetVolume();
  SoundDbg_Play(PHRASE_STARTUP);
  (void)StopAll();
  (void)StdPlayOn(CH_RUN, PHRASE_STARTUP);
}

/* 方向切替時: 走行中なら始動音からやり直す（SoundInitはしない） */
static void SoundRestartFromStartup(void)
{
  FanVolScale = 0U;
  _SetVolume();
  (void)StopAll();
  RunSound_Reset();
  SoundDbg_Play(PHRASE_STARTUP);
  (void)StdPlayOn(CH_RUN, PHRASE_STARTUP);
  PowerState = powerStart;
}

#ifdef SOUND_TEST_MENU
static uint8_t SoundTestActive = 1U; /* 1=音源確認メニュー, 0=通常(電源連動) */

/* USART1(VCP) から非ブロッキング1文字取得 */
static int Uart1_TryGetChar(char *out)
{
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != 0U) {
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);
  }
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) == 0U) {
    return 0;
  }
  *out = (char)(huart1.Instance->RDR & 0xFFu);
  return 1;
}

static void SoundTest_PrintDir(void)
{
  int vm = digitalRead_VM();
  printf("DIR status: %s Tail=%s vm=%d Power=%dmV armed=%u ok=%u nTailLED_gpio=%d\n",
         IsNormalDir ? "FWD" : "REV",
         IsNormalDir ? "OFF" : "ON",
         vm, PowerMV,
         (unsigned)DirRelatchArmed,
         (unsigned)DirPowerOk,
         TailLedGpioIsHigh());
}

static void SoundTest_PrintMenu(void)
{
  printf("\n=== Sound Test Menu (USART1) ===\n");
  printf("DIR now: %s  Tail=%s  Power=%dmV\n",
         IsNormalDir ? "FWD" : "REV",
         IsNormalDir ? "OFF" : "ON",
         PowerMV);
  printf("1: Startup     (P%d one-shot CH0)\n", PHRASE_STARTUP);
  printf("2: Run         (P%d loop CH0)\n", PHRASE_RUN_LOOP);
  printf("3: Radiator    (P%d loop CH0)\n", PHRASE_FAN_LOOP);
  printf("4: Run+Radiator overlay (CH0+CH1)\n");
  printf("0: Stop all\n");
  printf("d: Show direction status\n");
  printf("p: Print Power/DIR status now\n");
  printf("l: Blink nTailLED 3x (find which LED is ours)\n");
  printf("v: VM pin watch (toggle) — edges when Power OK (skip brownout)\n");
  printf("m: Measure POWER/VM edges (toggle)\n");
  printf("n: Normal operation (power-linked)\n");
  printf("h: Help\n");
  printf("> ");
}

static void SoundTest_EnterNormal(void)
{
  printf("-> Normal mode (power-linked). Press 't' for test menu.\n");
  StopAll();
  FanVolScale = 0U;
  _SetVolume();
  RunSound_Reset();
  SoundTestActive = 0U;
  if (PowerMV >= POWER_ON_TH) {
    SoundDbg_Play(PHRASE_STARTUP);
    (void)StdPlayOn(CH_RUN, PHRASE_STARTUP);
    PowerState = powerStart;
  } else {
    PowerState = powerIdle;
    printf("Waiting for power ON...\n");
  }
}

static void SoundTest_EnterTest(void)
{
  printf("-> Sound test menu\n");
  StopAll();
  FanVolScale = 0U;
  _SetVolume();
  RunSound_Reset();
  PowerState = powerIdle;
  SoundTestActive = 1U;
  SoundTest_PrintMenu();
}

static void SoundTest_Task(void)
{
  char c;

  if (Uart1_TryGetChar(&c) == 0) {
    return;
  }
  if (c == '\r' || c == '\n') {
    return;
  }
  printf("%c\n", c);

  switch (c) {
  case '0':
    SoundDbg_Stop("test menu");
    StopAll();
    FanVolScale = 0U;
    _SetVolume();
    break;
  case '1':
    StopAll();
    FanVolScale = 0U;
    _SetVolume();
    SoundDbg_Play(PHRASE_STARTUP);
    (void)StdPlayOn(CH_RUN, PHRASE_STARTUP);
    break;
  case '2':
    StopAll();
    FanVolScale = 0U;
    _SetVolume();
    SoundDbg_Loop(PHRASE_RUN_LOOP);
    (void)LoopOn(CH_RUN, PHRASE_RUN_LOOP);
    break;
  case '3':
    StopAll();
    FanVolScale = 0U;
    _SetVolume();
    SoundDbg_Loop(PHRASE_FAN_LOOP);
    (void)LoopOn(CH_RUN, PHRASE_FAN_LOOP);
    break;
  case '4':
    StopAll();
    FanVolScale = 255U;
    _SetVolume();
    SoundDbg_Loop(PHRASE_RUN_LOOP);
    (void)LoopOn(CH_RUN, PHRASE_RUN_LOOP);
    SoundDbg_Loop(PHRASE_FAN_LOOP);
    (void)LoopOn(CH_FAN, PHRASE_FAN_LOOP);
    break;
  case 'n':
  case 'N':
    SoundTest_EnterNormal();
    return;
  case 'd':
  case 'D':
    SoundTest_PrintDir();
    break;
  case 'p':
  case 'P':
    printf("Power=%dmV good=%u allow=%u hadOn=%u armed=%u lock=%u VM=%d Tail=%s\n",
           PowerMV,
           (unsigned)DirVmPowerGood,
           (unsigned)DirAllowArm,
           (unsigned)DirHadSolidOn,
           (unsigned)DirRelatchArmed,
           (unsigned)DirPostChangeLock,
           digitalRead_VM() ? 1 : 0,
           IsNormalDir ? "OFF" : "ON");
    if (PowerMV < DIR_POWER_ON_MV) {
      printf("  (DIR ready needs Power>=%dmV)\n", DIR_POWER_ON_MV);
    }
    break;
  case 'l':
  case 'L':
    {
      int i;
      printf("Blink nTailLED (PB7) 3 times — watch which lamp toggles\n");
      for (i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(nTailLED_GPIO_Port, nTailLED_Pin, GPIO_PIN_RESET);
        printf("  [%d] LED drive ON (gpio L) gpio=%d\n", i + 1, TailLedGpioIsHigh());
        HAL_Delay(300);
        HAL_GPIO_WritePin(nTailLED_GPIO_Port, nTailLED_Pin, GPIO_PIN_SET);
        printf("  [%d] LED drive OFF (gpio H) gpio=%d\n", i + 1, TailLedGpioIsHigh());
        HAL_Delay(300);
      }
      SetTailLED(IsNormalDir);
      printf("restored TAIL %s gpio=%d\n",
             IsNormalDir ? "OFF" : "ON", TailLedGpioIsHigh());
    }
    break;
  case 'm':
  case 'M':
    DirMeasEnable = (DirMeasEnable != 0U) ? 0U : 1U;
    DirMeasLastPwr = -1;
    DirMeasLastVm = -1;
    DirMeasT0 = HAL_GetTick();
    printf("MEAS %s (reverse power and/or whistle; watch T+..ms lines)\n",
           DirMeasEnable ? "ON" : "OFF");
    break;
  case 'v':
  case 'V':
    DirVmWatchEnable = (DirVmWatchEnable != 0U) ? 0U : 1U;
    DirVmWatchLast = -1;
    DirVmWatchBrownout = 0U;
    printf("VM watch %s (VMIN edges only when Power>=%dmV)\n",
           DirVmWatchEnable ? "ON" : "OFF", DIR_POWER_ON_MV);
    break;
  case 'h':
  case 'H':
  case '?':
    SoundTest_PrintMenu();
    return;
  default:
    printf("unknown key\n");
    SoundTest_PrintMenu();
    return;
  }
  printf("> ");
}
#endif /* SOUND_TEST_MENU */

/* powerON中: 走行音は常時ループ。間隔毎にラジエターをCH1で30秒重ね（両端2秒フェード） */

/* 電源復帰後の方向確定。変化は早く、同一は長く待ってから武装解除。 */
/* After recover: CHANGED fast; SAME waits long so late VM flip is caught.
 * Also print keep/arm so missed return reverse is diagnosable. */

static void DirMeas_Update(uint8_t off_like)
{
  int vm;
  int8_t pwr;
  uint32_t dt;

  if (DirMeasEnable == 0U) {
    return;
  }
  pwr = (off_like != 0U) ? (int8_t)0 : (int8_t)1;
  vm = digitalRead_VM() ? 1 : 0;
  dt = HAL_GetTick() - DirMeasT0;
  if (DirMeasLastPwr < 0) {
    DirMeasLastPwr = pwr;
    DirMeasLastVm = (int8_t)vm;
    printf("MEAS start Power=%dmV VM=%d\n", PowerMV, vm);
    return;
  }
  if (pwr != DirMeasLastPwr) {
    printf("T+%lums POWER %s->%s %dmV\n",
           (unsigned long)dt,
           DirMeasLastPwr ? "ON" : "OFF",
           pwr ? "ON" : "OFF",
           PowerMV);
    DirMeasLastPwr = pwr;
  }
  if (vm != (int)DirMeasLastVm) {
    printf("T+%lums VM %d->%d Power=%dmV\n",
           (unsigned long)dt,
           (int)DirMeasLastVm, vm, PowerMV);
    DirMeasLastVm = (int8_t)vm;
  }
}

/* VMIN edge log: skip true brownout (<OFF_MV). Hyst keeps logging across 4500–4800. */
static void DirVmWatch_Update(void)
{
  int vm;

  if (DirVmWatchEnable == 0U) {
    return;
  }
  if (DirVmPowerGood == 0U) {
    if (DirVmWatchBrownout == 0U) {
      DirVmWatchBrownout = 1U;
      printf("VM ignore (brownout Power=%dmV)\n", PowerMV);
      DirVmWatchLast = -1; /* resync after recover */
    }
    return;
  }
  DirVmWatchBrownout = 0U;
  vm = digitalRead_VM() ? 1 : 0;
  if (DirVmWatchLast < 0) {
    DirVmWatchLast = (int8_t)vm;
    printf("VM now=%d Power=%dmV (watch ON, 'v' to toggle)\n", vm, PowerMV);
    return;
  }
  if (vm != (int)DirVmWatchLast) {
    printf("VM %d->%d Power=%dmV t=%lums\n",
           (int)DirVmWatchLast, vm, PowerMV,
           (unsigned long)HAL_GetTick());
    DirVmWatchLast = (int8_t)vm;
  }
}


/* Finish a direction update (VM-based or power-gap toggle). */
static void DirCommitDir(int prev_dir, int new_dir, const char *why)
{
  IsNormalDir = new_dir;
  /* VM polarity: FWD=VM0, REV=VM1 */
  DirLastVM = (uint8_t)(new_dir ? 0U : 1U);
  DirHoldMs = 0;
  LastLoggedDir = IsNormalDir;
#ifdef DIR_DEBUG
  printf("DIR %s->%s Tail=%s (%s) Power=%dmV\n",
         prev_dir ? "FWD" : "REV",
         IsNormalDir ? "FWD" : "REV",
         IsNormalDir ? "OFF" : "ON",
         why,
         PowerMV);
#else
  (void)why;
#endif
  DirApplyTailLed(why);
  DirRelatchArmed = 0U;
  DirArmLogged = 0U;
  DirRecoverMs = 0;
  DirVmStableMs = 0;
  DirVmCand = 0xFFu;
  DirPowerOffMs = 0;
  DirAllowArm = 0U;
  DirPostChangeLock = 1U;
  DirOnSettleMs = 0;
  DirInitPending = 0U;
  DirVinSwingOk = 0U;
  DirFollowCand = 0xFFu;
  DirFollowStableMs = 0;
  DirQaHoldUntil = HAL_GetTick() + (uint32_t)DIR_POST_HOLD_MS;
  if (PowerMV > 0) {
    DirPowerPeakMv = (uint16_t)PowerMV;
  }
  /* 実際に方向が変わったら始動音から再開（電源ON中のみ） */
  if (prev_dir != new_dir &&
      (PowerState == powerON || PowerState == powerStart)) {
    SoundRestartFromStartup();
  }
}

/* Call only while armed and DirVmPowerGood.
 * Absolute direction from VM (not toggle). Polarity: VM=1→REV, VM=0→FWD.
 * Asymmetric timing (inverted from old VM=1=FWD mapping):
 *  FWD→REV: trust VM=1. Do NOT force while VM still 0.
 *  REV→FWD: trust early VM=0 (late VM=1 is a lie on return). */
static void DirTryLatchWhileArmed(void)
{
  int vm_now;
  int prev_dir;
  uint8_t vm_u8;

  if (DirVmPowerGood == 0U) {
    DirVmCand = 0xFFu;
    DirVmStableMs = 0;
    return;
  }

  if (DirRecoverMs < 0xFFFFu) {
    DirRecoverMs = (uint16_t)(DirRecoverMs + POWER_ADC_PERIOD_MS);
  }

  vm_now = digitalRead_VM();
  vm_u8 = (uint8_t)(vm_now ? 1U : 0U);
  if (DirVmCand != vm_u8) {
    DirVmCand = vm_u8;
    DirVmStableMs = 0;
  } else if (DirVmStableMs < 0xFFFFu) {
    DirVmStableMs = (uint16_t)(DirVmStableMs + POWER_ADC_PERIOD_MS);
  }

  prev_dir = IsNormalDir ? 1 : 0;

  if (prev_dir != 0) {
    /* Was FWD: need REV (VM=1). Early VM=0 is ramp — never force while VM==0. */
    if (vm_u8 != 0U && DirVmStableMs >= DIR_VM_CHANGE_MS) {
      DirCommitDir(prev_dir, 0, "VM-REV");
      return;
    }
    if (DirRecoverMs >= DIR_REV_FORCE_MS && vm_u8 != 0U) {
      DirCommitDir(prev_dir, 0, "force-REV");
      return;
    }
    if (DirRecoverMs >= DIR_REV_FORCE_HARD_MS) {
      if (vm_u8 != 0U) {
        DirCommitDir(prev_dir, 0, "force-REV-hard");
      } else if (DirVmStableMs >= DIR_VM_CHANGE_MS) {
        /* VM=0=正転のまま — スロットル下げ等の誤武装。方向は変えない */
        DirRelatchArmed = 0U;
        DirArmLogged = 0U;
        DirRecoverMs = 0;
        DirVmStableMs = 0;
        DirVmCand = 0xFFu;
        DirAllowArm = 1U;
        DirPowerPeakMv = (uint16_t)PowerMV;
      }
    }
  } else {
    /* Was REV: take stable VM=0 as FWD. Do NOT force FWD early —
     * reverse recover often lies with VM=0 and wiped the taillight. */
    if (vm_u8 == 0U && DirVmStableMs >= DIR_VM_CHANGE_MS &&
        DirRecoverMs >= DIR_FWD_FORCE_MS) {
      DirCommitDir(prev_dir, 1, "VM-FWD");
      return;
    }
    if (DirRecoverMs >= (DIR_FWD_FORCE_MS + 500U) && vm_u8 == 0U &&
        DirVmStableMs >= DIR_VM_CHANGE_MS) {
      DirCommitDir(prev_dir, 1, "force-FWD");
      return;
    }
    if (DirRecoverMs >= (DIR_FWD_FORCE_MS + 500U) &&
        vm_u8 != 0U && DirVmStableMs >= DIR_VM_CHANGE_MS) {
      /* Confirmed still REV after gap — drop arm, keep taillight ON */
      DirRelatchArmed = 0U;
      DirArmLogged = 0U;
      DirRecoverMs = 0;
      DirVmStableMs = 0;
      DirVmCand = 0xFFu;
      DirAllowArm = 1U;
      DirPowerPeakMv = (uint16_t)PowerMV;
    }
  }
}





static void RunSound_Task(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed = now - RunModeSince;
  uint32_t remain;
  uint8_t scale;

  if (RunSoundState == runMainLoop) {
    if (elapsed >= RUN_FAN_INTERVAL_MS) {
      SoundDbg_Loop(PHRASE_FAN_LOOP);
      FanCountdownLastSec = -1;
      RunSound_ApplyFanVol(0U);
      (void)LoopOn(CH_FAN, PHRASE_FAN_LOOP);
      RunSoundState = runFanLoop;
      RunModeSince = now;
      elapsed = 0;
    } else {
      remain = RUN_FAN_INTERVAL_MS - elapsed;
      RunSound_Countdown(remain, "next");
      return;
    }
  }

  /* runFanLoop */
  scale = RunSound_FanFadeScale(elapsed);
  RunSound_ApplyFanVol(scale);
  if (elapsed >= RUN_FAN_DURATION_MS) {
    SoundDbg_Stop("fan overlay end");
    (void)StopOn(CH_FAN);
    RunSound_ApplyFanVol(0U);
    RunSoundState = runMainLoop;
    RunModeSince = now;
    FanCountdownLastSec = -1;
  } else {
    remain = RUN_FAN_DURATION_MS - elapsed;
    if (elapsed < RUN_FAN_FADE_MS) {
      RunSound_Countdown(remain, "fade-in");
    } else if (elapsed >= (RUN_FAN_DURATION_MS - RUN_FAN_FADE_MS)) {
      RunSound_Countdown(remain, "fade-out");
    } else {
      RunSound_Countdown(remain, "play");
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  // 受信割り込み開始
   //UART2_Start_Receive_IT();
   setvbuf(stdout, NULL, _IONBF, 0); // printfバッファ無効化
   printf("%s Build %s\n", FW_PRODUCT_NAME, FW_BUILD_NUMBER);
   QA_Init();   // 初期化（IO初期状態、TIM14割り込み開始）
   HAL_ADCEx_Calibration_Start(&hadc1);
   adc_tick = HAL_GetTick();

   SOUND_CS_H();
   delay_ms(100);
   LastPowerState=-1;


   //SPI1_SanityTest();

   //==================================================
   // Sound Check
   //==================================================
   SoundInit();
   //Sound_PlayLikeTool();
   // 初期化直後の推奨設定
//	SetAVol(255);          // 全体=最大
//	SetCVolAll(255);       // 各ch=最大

#if 0
	// AとBを同時発音
	PlayOn(0, 0);          // CH0 ← フレーズ0
	PlayOn(1, 1);          // CH1 ← フレーズ1

	// Bを少し絞る（約半分）
	SetCVol(1, 128);

	// 空いているチャネルに自動割当
	PlayAuto(2);

	// 停止
	StopOn(0);
	StopAll();
	// 初期化直後の推奨設定
	SetAVol(255);          // 全体=最大
	SetCVolAll(255);       // 各ch=最大
#else
	_SetVolume();
	//LoopOn(0, PHRASE_RUN_LOOP);
#endif
#ifdef SOUND_TEST_MENU
   printf("SOUND_TEST_MENU: press 1/2/3/4/0 to test, 'n' for normal\n");
   SoundTest_PrintMenu();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  QA_Task();

	  //TailLampの処理==============================================
	  SetTailLED(IsNormalDir);

	  //ADCの取得（尾灯方向用 OFF 判定のため 5ms 周期）===============
	    if (Elapsed(adc_tick) >= POWER_ADC_PERIOD_MS)
	    {
	        adc_tick += POWER_ADC_PERIOD_MS;

int val = AdcRead();
	        if(val>=0){
	        	uint8_t became_armed;
	        	uint8_t off_like;
	        	PowerMV=GetPower_mV(val);

	        	if ((int)DirPowerPeakMv < PowerMV) {
	        		DirPowerPeakMv = (uint16_t)PowerMV;
	        	}

	        	off_like = 0U;
	        	if (PowerMV < DIR_POWER_OFF_MV) {
	        		off_like = 1U;
	        	} else if (DirPowerPeakMv > 0U &&
	        		   DirPowerPeakMv <= (uint16_t)DIR_DROP_ARM_MAX_PEAK) {
	        		/* DROP武装は低電圧peak時のみ。
	        		 * 高電圧からスロットルを下げただけでは武装しない。 */
	        		uint16_t drop_need = (uint16_t)DIR_DROP_ARM_MV;
	        		uint16_t drop_pct = (uint16_t)(((uint32_t)DirPowerPeakMv * (uint32_t)DIR_DROP_ARM_PCT) / 100U);
	        		if (drop_pct > drop_need) {
	        			drop_need = drop_pct;
	        		}
	        		if ((uint16_t)PowerMV + drop_need <= DirPowerPeakMv) {
	        			off_like = 1U;
	        		}
	        	}

	        	/* 走行中はpeakを現在値へ追従（スロットル下げ後に古い高peakが残らない） */
	        	if (off_like == 0U && DirRelatchArmed == 0U && DirVmPowerGood != 0U) {
	        		DirPowerPeakMv = (uint16_t)PowerMV;
	        	}

	        	DirMeas_Update(off_like);

	        	/* VM trust hysteresis: set at ON_MV, clear only below OFF_MV. */
	        	if (PowerMV >= DIR_POWER_ON_MV) {
	        		if (DirVmPowerGood == 0U) {
	        			DirVmPowerGood = 1U;
	        			DirRecoverMs = 0;
	        			DirVmCand = 0xFFu;
	        			DirVmStableMs = 0;
#ifdef DIR_DEBUG
	        			printf("DIR power-good Power=%dmV\n", PowerMV);
#endif
	        		}
	        		DirHadSolidOn = 1U;
	        		/* Become ready as soon as solid ON (unless post-change locked) */
	        		if (DirPostChangeLock == 0U && DirRelatchArmed == 0U) {
#ifdef DIR_DEBUG
	        			if (DirAllowArm == 0U) {
	        				printf("DIR ready (solid ON) Power=%dmV\n", PowerMV);
	        			}
#endif
	        			DirAllowArm = 1U;
	        		}
	        	} else if (PowerMV < DIR_POWER_OFF_MV) {
	        		if (DirVmPowerGood != 0U) {
	        			DirVmPowerGood = 0U;
	        			DirRecoverMs = 0;
	        			DirVmCand = 0xFFu;
	        			DirVmStableMs = 0;
	        		}
	        		/* Release post-change lock on deep OFF so next gap can arm.
	        		 * Do NOT set AllowArm here — that falsely armed on boot/power-up. */
	        		DirPostChangeLock = 0U;
	        		DirOnSettleMs = 0;
	        		/* Next solid ON: absolute DIR from VM (PowerON判定) */
	        		DirInitPending = 1U;
	        		DirInitCand = 0xFFu;
	        		DirInitStableMs = 0;
	        	}

	        	/* PowerON: VM安定後に絶対方向を確定（VM=1→反転, VM=0→正転） */
	        	if (DirInitPending != 0U && DirVmPowerGood != 0U) {
	        		uint8_t vm_u8 = (uint8_t)(digitalRead_VM() ? 1U : 0U);
	        		if (DirInitCand != vm_u8) {
	        			DirInitCand = vm_u8;
	        			DirInitStableMs = 0;
	        		} else if (DirInitStableMs < 0xFFFFu) {
	        			DirInitStableMs = (uint16_t)(DirInitStableMs + POWER_ADC_PERIOD_MS);
	        		}
	        		if (DirInitStableMs >= DIR_VM_CHANGE_MS) {
	        			int prev = IsNormalDir ? 1 : 0;
	        			int neu = DirFromVm((int)vm_u8);
	        			DirCommitDir(prev, neu, "power-ON-VM");
	        			DirRelatchArmed = 0U;
	        		}
	        	} else if (DirVmPowerGood != 0U && DirPostChangeLock == 0U) {
	        		uint32_t now = HAL_GetTick();
	        		/* Vin振れ監視: 汽笛はVMのみ変化、実反転は電源にも振れが出やすい */
	        		if (DirVinWinMs == 0U) {
	        			DirVinWinMax = (uint16_t)PowerMV;
	        			DirVinWinMin = (uint16_t)PowerMV;
	        		} else {
	        			if ((int)DirVinWinMax < PowerMV) {
	        				DirVinWinMax = (uint16_t)PowerMV;
	        			}
	        			if ((int)DirVinWinMin > PowerMV) {
	        				DirVinWinMin = (uint16_t)PowerMV;
	        			}
	        		}
	        		if (DirVinWinMs < 0xFFFFu) {
	        			DirVinWinMs = (uint16_t)(DirVinWinMs + POWER_ADC_PERIOD_MS);
	        		}
	        		/* 振れは検出したら hold(800)+follow(500) より長く保持する。
	        		 * 静かな 300ms 窓で落とすと、切替時の一瞬の落ち込みを逃す。 */
	        		if (DirVinWinMax >= DirVinWinMin &&
	        		    (uint16_t)(DirVinWinMax - DirVinWinMin) >= (uint16_t)DIR_FOLLOW_VIN_SWING_MV) {
	        			DirVinSwingOk = 1U;
	        			DirVinSwingUntil = now + (uint32_t)DIR_FOLLOW_VIN_KEEP_MS;
	        		}
	        		if (DirVinWinMs >= DIR_FOLLOW_VIN_WIN_MS) {
	        			DirVinWinMs = 0;
	        		}
	        		if (DirVinSwingOk != 0U &&
	        		    (int32_t)(now - DirVinSwingUntil) >= 0) {
	        			DirVinSwingOk = 0U;
	        		}

	        		if (CmdMode || IsWhistle ||
	        		    ((int32_t)(now - DirQaHoldUntil) < 0)) {
	        			DirFollowCand = 0xFFu;
	        			DirFollowStableMs = 0;
	        		} else if (DirVinSwingOk != 0U) {
	        			/* 手動反転など: Vin振れあり + VM安定でDIR追従。汽笛は振れ無しで除外。 */
	        			uint8_t vm_u8 = (uint8_t)(digitalRead_VM() ? 1U : 0U);
	        			int cur = IsNormalDir ? 1 : 0;
	        			int neu = DirFromVm((int)vm_u8);
	        			if (DirFollowCand != vm_u8) {
	        				DirFollowCand = vm_u8;
	        				DirFollowStableMs = 0;
	        			} else if (DirFollowStableMs < 0xFFFFu) {
	        				DirFollowStableMs = (uint16_t)(DirFollowStableMs + POWER_ADC_PERIOD_MS);
	        			}
	        			if (neu != cur && DirFollowStableMs >= DIR_VM_FOLLOW_MS) {
	        				DirCommitDir(cur, neu, "VM-follow");
	        				DirRelatchArmed = 0U;
	        				DirFollowCand = 0xFFu;
	        				DirFollowStableMs = 0;
	        				DirVinSwingOk = 0U;
	        			}
	        		} else {
	        			DirFollowCand = 0xFFu;
	        			DirFollowStableMs = 0;
	        		}
	        	} else {
	        		DirFollowCand = 0xFFu;
	        		DirFollowStableMs = 0;
	        	}

	        	DirVmWatch_Update();

#ifdef POWER_VOLT_LOG
	        	{
	        		uint32_t now = HAL_GetTick();
	        		int vm_now = digitalRead_VM() ? 1 : 0;
	        		int want = DirFromVm(vm_now);
	        		uint16_t swing = 0;
	        		int32_t hold = (int32_t)(DirQaHoldUntil - now);
	        		const char *blk;
	        		if (DirVinWinMax >= DirVinWinMin) {
	        			swing = (uint16_t)(DirVinWinMax - DirVinWinMin);
	        		}
	        		if (hold < 0) {
	        			hold = 0;
	        		}
	        		if (CmdMode || IsWhistle) {
	        			blk = "whistle";
	        		} else if (hold > 0) {
	        			blk = "hold";
	        		} else if (DirVinSwingOk == 0U) {
	        			blk = "vin";
	        		} else if (want == (IsNormalDir ? 1 : 0)) {
	        			blk = "same";
	        		} else if (DirFollowStableMs < DIR_VM_FOLLOW_MS) {
	        			blk = "wait";
	        		} else {
	        			blk = "ok";
	        		}
	        		if (DirLogLastVm < 0) {
	        			DirLogLastVm = vm_now;
	        			DirLogVmTick = now;
	        		} else if (vm_now != DirLogLastVm) {
	        			printf("VM %d->%d DT=%lums Vin=%dmV swing=%umV/%d Dir=%s want=%s blk=%s\n",
	        			       DirLogLastVm, vm_now,
	        			       (unsigned long)(now - DirLogVmTick),
	        			       PowerMV, (unsigned)swing, DIR_FOLLOW_VIN_SWING_MV,
	        			       IsNormalDir ? "FWD" : "REV",
	        			       want ? "FWD" : "REV",
	        			       blk);
	        			DirLogLastVm = vm_now;
	        			DirLogVmTick = now;
	        		}
	        		if ((now - PowerVoltLogMs) >= 1000U) {
	        			PowerVoltLogMs = now;
	        			printf("Vin=%dmV VM=%d Dir=%s want=%s state=%d DT=%lums swing=%umV/%d blk=%s (ON>=%d OFF<=%d)\n",
	        			       PowerMV,
	        			       vm_now,
	        			       IsNormalDir ? "FWD" : "REV",
	        			       want ? "FWD" : "REV",
	        			       PowerState,
	        			       (unsigned long)(now - DirLogVmTick),
	        			       (unsigned)swing, DIR_FOLLOW_VIN_SWING_MV,
	        			       blk,
	        			       POWER_ON_TH, POWER_OFF_TH);
	        		}
	        	}
#endif

#ifdef DIR_DEBUG
	        	/* Periodic power so "power ON but no ready" is diagnosable */
	        	{
	        		uint32_t now = HAL_GetTick();
	        		if ((now - DirPowerLogMs) >= 1000U) {
	        			DirPowerLogMs = now;
	        			printf("Power=%dmV good=%u allow=%u\n",
	        			       PowerMV,
	        			       (unsigned)DirVmPowerGood,
	        			       (unsigned)DirAllowArm);
	        			if (DirVmPowerGood == 0U && DirWaitLogged == 0U) {
	        				DirWaitLogged = 1U;
	        				printf("DIR wait: need Power>=%dmV for ready\n",
	        				       DIR_POWER_ON_MV);
	        			}
	        		}
	        		if (DirVmPowerGood != 0U) {
	        			DirWaitLogged = 0U;
	        		}
	        	}
#endif

	        	/* After CHANGE: wait for continuous solid ON before next arm. */
	        	if (DirPostChangeLock != 0U) {
	        		DirRelatchArmed = 0U;
	        		DirAllowArm = 0U;
	        		if (DirVmPowerGood != 0U) {
	        			if (DirOnSettleMs < 0xFFFFu) {
	        				DirOnSettleMs = (uint16_t)(DirOnSettleMs + POWER_ADC_PERIOD_MS);
	        			}
	        			if (DirOnSettleMs >= DIR_ON_SETTLE_MS) {
	        				DirPostChangeLock = 0U;
	        				DirAllowArm = 1U;
	        				DirOnSettleMs = 0;
	        				if ((int)DirPowerPeakMv < PowerMV) {
	        					DirPowerPeakMv = (uint16_t)PowerMV;
	        				}
	        				DirArmLogged = 0U;
	        			}
	        		} else {
	        			DirOnSettleMs = 0;
	        		}
	        	}

	        	if (off_like != 0U) {
	        		DirPowerOk = 0U;
	        		became_armed = 0U;
	        		if (DirVmPowerGood == 0U) {
	        			DirRecoverMs = 0;
	        			DirVmCand = 0xFFu;
	        			DirVmStableMs = 0;
	        		}
	        		if (DirAllowArm != 0U && DirPostChangeLock == 0U && DirHadSolidOn != 0U) {
	        			if (DirPowerOffMs < 0xFFFFu) {
	        				DirPowerOffMs = (uint16_t)(DirPowerOffMs + POWER_ADC_PERIOD_MS);
	        			}
	        			if (DirPowerOffMs >= DIR_POWER_OFF_MS) {
	        				if (DirRelatchArmed == 0U) {
	        					became_armed = 1U;
	        				}
	        				DirRelatchArmed = 1U;
	        			}
	        		} else {
	        			DirPowerOffMs = 0;
	        			DirRelatchArmed = 0U;
	        		}
	        		DirPowerWasOff = 1U;
	        		if (became_armed != 0U && DirArmLogged == 0U) {
	        			DirArmLogged = 1U;
#ifdef DIR_DEBUG
	        			printf("DIR arm Power=%dmV peak=%umV\n",
	        			       PowerMV, (unsigned)DirPowerPeakMv);
#endif
	        			/* Decide on recover (absolute VM), not here */
	        		}
	        		if (DirRelatchArmed != 0U && DirVmPowerGood != 0U) {
	        			DirTryLatchWhileArmed();
	        		}
	        	} else if (DirVmPowerGood != 0U) {
	        		DirPowerOk = 1U;
	        		if (DirPowerWasOff != 0U) {
	        			DirPowerWasOff = 0U;
	        		}
	        		DirPowerOffMs = 0;
	        		if (DirRelatchArmed != 0U) {
	        			DirTryLatchWhileArmed();
	        		} else if (DirPostChangeLock == 0U) {
#ifdef DIR_DEBUG
	        			if (DirAllowArm == 0U) {
	        				printf("DIR ready (solid ON) Power=%dmV\n", PowerMV);
	        			}
#endif
	        			DirArmLogged = 0U;
	        			DirAllowArm = 1U;
	        			DirHadSolidOn = 1U;
	        			if ((int)DirPowerPeakMv < PowerMV) {
	        				DirPowerPeakMv = (uint16_t)PowerMV;
	        			}
	        		}
	        	} else {
	        		DirPowerWasOff = 1U;
	        		DirPowerOffMs = 0;
	        	}
#ifdef POWER_CHECK_MODE
	        	printf("Power=%dmV armed=%u ok=%u\n",
	        	       PowerMV, (unsigned)DirRelatchArmed, (unsigned)DirPowerOk);
#endif
	        }
	    }
#ifdef SOUND_TEST_MENU
    	if (SoundTestActive != 0U) {
    		/* 音源確認メニュー */
    		SoundTest_Task();
    	} else {
    		char c;
    		/* 通常動作中: 't' でテストメニューへ戻る */
    		if (Uart1_TryGetChar(&c) != 0) {
    			if (c == 't' || c == 'T') {
    				SoundTest_EnterTest();
    			}
    		}
    		if (SoundTestActive == 0U) {
    			/* 電源連動: 起動音=P2(CH0)、走行=P1ループ(CH0)、ラジエター=P0重ね(CH1) */
    			switch(PowerState){
    			case powerIdle:
    				if(PowerMV>=POWER_ON_TH){
    					SoundDbg_Play(PHRASE_STARTUP);
    					StdPlayOn(0, PHRASE_STARTUP);
    					PowerState=powerStart;
    				}
    				break;
    			case powerStart:
    				if(!IsPlaying(0)){
    					SoundDbg_Stop("startup end");
    					RunSound_StartMainLoop();
    					PowerState=powerON;
    				}
    				if(PowerMV<=POWER_OFF_TH){
    					SoundDbg_Stop("power off");
    					StopAll();
    					RunSound_Reset();
    					PowerState=powerStop;
    				}
    				break;
    			case powerON:
    				RunSound_Task();
    				if(PowerMV<=POWER_OFF_TH){
    					SoundDbg_Stop("power off");
    					StopAll();
    					RunSound_Reset();
    					PowerState=powerStop;
    				}
    				break;
    			case powerStop:
    				if(!IsPlaying(0)){
    					SoundDbg_Stop("idle");
    					PowerState=powerIdle;
    				}
    				if(PowerMV>=POWER_ON_TH){
    					SoundDbg_Stop("re-power");
    					StopAll();
    					RunSound_Reset();
    					SoundDbg_Play(PHRASE_STARTUP);
    					StdPlayOn(0, PHRASE_STARTUP);
    					PowerState=powerStart;
    				}
    				break;
    			}
    		}
    	}
#else
    	/* 電源連動: 起動音=P2(CH0)、走行=P1ループ(CH0)、ラジエター=P0重ね(CH1) */
    	switch(PowerState){
    	case powerIdle:
    		if(PowerMV>=POWER_ON_TH){
    			SoundPowerOn_Start();
    			PowerState=powerStart;
    		}
    		break;
    	case powerStart:
    	    if(!IsPlaying(0)){
    	    	SoundDbg_Stop("startup end");
    	    	RunSound_StartMainLoop();
    			PowerState=powerON;
    	    }
    		if(PowerMV<=POWER_OFF_TH){
    			SoundDbg_Stop("power off");
    			StopAll();
    			RunSound_Reset();
    			PowerState=powerStop;
    		}
    		break;
    	case powerON:
    		RunSound_Task();
    		if(PowerMV<=POWER_OFF_TH){
    			SoundDbg_Stop("power off");
    			StopAll();
    			RunSound_Reset();
    			PowerState=powerStop;
    		}
    		break;

    	case powerStop:
    	    if(!IsPlaying(0)){
    			SoundDbg_Stop("idle");
    			PowerState=powerIdle;
    	    }
    		if(PowerMV>=POWER_ON_TH){
    			SoundDbg_Stop("re-power");
    			RunSound_Reset();
    			SoundPowerOn_Start();
    			PowerState=powerStart;
    		}
    		break;
    	}
#endif /* SOUND_TEST_MENU */
#if 0
  	  if(LastPowerState!=PowerState){
  		  if((unsigned)PowerState <
  		     (sizeof(PowerStateName)/sizeof(PowerStateName[0]))){
  		  	printf("State=%s,Power=%dmV\n",
  		  	       PowerStateName[PowerState],PowerMV);
  		  }
  		  LastPowerState=PowerState;
  	  }
#endif


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 999;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 47;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IO2_Pin|NCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IO0_Pin|nTailLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : VMIN_Pin VPIN_Pin */
  GPIO_InitStruct.Pin = VMIN_Pin|VPIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IO2_Pin NCS_Pin */
  GPIO_InitStruct.Pin = IO2_Pin|NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CBUSY_Pin */
  GPIO_InitStruct.Pin = CBUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CBUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IO0_Pin nTailLED_Pin */
  GPIO_InitStruct.Pin = IO0_Pin|nTailLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
