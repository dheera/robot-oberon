
#include "stdlib.h"
#include "Arduino.h"

unsigned char dut[4]; ///< PWM value
unsigned char conter; ///< Counter every 1ms up plus 1, range 0-255
unsigned char PWMFlag;///< PWM logo, used to determine which one you should use IO

void TIM5_IRQHandler(void)
{              
  TIM5->SR = ~0x0001;
  conter++;
    
  if(PWMFlag & 0x01){
    if(!(PWMFlag & 0x10)){
      GPIO_ResetBits(GPIOC, GPIO_Pin_11);
    }else{
      if(conter<dut[0]){
        GPIO_SetBits(GPIOC, GPIO_Pin_11);
      }else if(conter!=255){
        GPIO_ResetBits(GPIOC, GPIO_Pin_11);
      }
    }
   }else{
      if(!(PWMFlag & 0x10)){
        GPIO_ResetBits(GPIOC, GPIO_Pin_12);
      }else{
        if(conter<dut[0]){
          GPIO_SetBits(GPIOC, GPIO_Pin_12);
        }else if(conter!=255){
          GPIO_ResetBits(GPIOC, GPIO_Pin_12);
        }
      }
    }        
        
  if(PWMFlag & 0x02){
    if(!(PWMFlag & 0x20)){
      GPIO_ResetBits(GPIOA, GPIO_Pin_11);
    }else{
      if(conter<dut[1]){
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
      }else if(conter!=255){
        GPIO_ResetBits(GPIOA, GPIO_Pin_11);
      }
    }
  }else{
      if(!(PWMFlag & 0x20)){
        GPIO_ResetBits(GPIOC, GPIO_Pin_10);
      }else{
        if(conter<dut[1]){
          GPIO_SetBits(GPIOC, GPIO_Pin_10);
        }else if(conter!=255){
      GPIO_ResetBits(GPIOC, GPIO_Pin_10);
        }
      }
    }
        
    if(PWMFlag & 0x04){
      if(!(PWMFlag & 0x40)){
        GPIO_ResetBits(GPIOB,GPIO_Pin_9);
      }else{
        if(conter<dut[2]){
          GPIO_SetBits(GPIOB,GPIO_Pin_9);
        }else if(conter!=255){
          GPIO_ResetBits(GPIOB,GPIO_Pin_9);
        }
      }  
    }else{
      if(!(PWMFlag & 0x40)){
        GPIO_ResetBits(GPIOB, GPIO_Pin_8);
      }else{
        if(conter<dut[2]){
            GPIO_SetBits(GPIOB, GPIO_Pin_8);
        }else if(conter!=255){
            GPIO_ResetBits(GPIOB, GPIO_Pin_8);
        }
      }
    }
        
    if(PWMFlag & 0x08)
    {
      if(!(PWMFlag & 0x80)){
        GPIO_ResetBits(GPIOB,GPIO_Pin_5);
      }else{
        if(conter<dut[3]){
          GPIO_SetBits(GPIOB,GPIO_Pin_5);
        }else if(conter!=255){
          GPIO_ResetBits(GPIOB,GPIO_Pin_5);
        }
      }
    }else{
      if(!(PWMFlag & 0x80)){
        GPIO_ResetBits(GPIOD,GPIO_Pin_2);
      }else{
        if(conter<dut[3]){
          GPIO_SetBits(GPIOD,GPIO_Pin_2);
        }else if(conter!=255){
          GPIO_ResetBits(GPIOD,GPIO_Pin_2);
        }
      }
    }        
  
}

void TIM_Configuration(void)
{ 
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  NVIC_InitTypeDef NVIC_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;        
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;      
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     
  NVIC_Init(&NVIC_InitStructure);               

  TIM_TimeBaseStructure.TIM_Period = 9;             
  TIM_TimeBaseStructure.TIM_Prescaler = 27;           
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;         
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0X0;    
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);   
   
  TIM_ClearFlag(TIM5, TIM_FLAG_Update);         
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);  
  TIM_Cmd(TIM5, ENABLE);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
}

void PWMStart(int channel)
{
  //PWMFlag |= motorIO[channel].pwmflag2;
  PWMFlag |= (0x10<<channel);
}

void PWMStop(int channel)
{
  //PWMFlag &= ~(motorIO[channel].pwmflag2);
  PWMFlag &= ~(0x10<<channel);
}

void PWMInit(unsigned char digitalPin) 
{
  TIM_Configuration();
  pinMode(digitalPin,OUTPUT);
  
  if(digitalPin==8){
    PWMFlag |= 0x01;
  }else if(digitalPin==23){
    PWMFlag &= 0xfe;
  }else if(digitalPin==7){
    PWMFlag |= 0x02;
  }else if(digitalPin==9){
    PWMFlag &= 0xfd;
  }else if(digitalPin==24){
    PWMFlag |= 0x04;
  }else if(digitalPin==14){
    PWMFlag &= 0xfb;
  }else if(digitalPin==4){
    PWMFlag |= 0x08;
  }else if(digitalPin==25){
    PWMFlag &= 0xf7;
  }
}

const int motorDirPin[4][2] = { //Forward, Backward
/*Motor-driven IO ports*/
  {8,23},
  {7,9},
  {24,14},
  {4,25}
};


const double motorPidParam[3]={0.1,0.8,0.05};/*Encoder V1.0,160rdint motorSpeed[4] = {-200,200,400,-400};/min ;19500/min; 32:1,Kr=3.5*/
int motorSpeed[4] = {0,0,0,0};
void setup() {
   delay(1000);
   Serial1.begin(115200);

     for(int j=0;j<4;j++){
       pinMode(motorDirPin[j][0], OUTPUT);
       digitalWrite(motorDirPin[j][0], LOW);
       PWMInit(motorDirPin[j][1]);
       PWMStart(j);
    }
   
   Serial1.println("# start");
}

unsigned long currentTime;
unsigned long lastSetTime;

char buf[256];
unsigned int i=0;

void set(uint8_t channel, int16_t value) {
  //if(value == 0) value = 1;
  if(value>=0) {
       pinMode(motorDirPin[channel][0], OUTPUT);
       digitalWrite(motorDirPin[channel][0], LOW);
       dut[channel] = 0xFF & ((value) >> 7);
       if(dut[channel] == 0) {
         PWMStop(channel);
         pinMode(motorDirPin[channel][0], OUTPUT);
         digitalWrite(motorDirPin[channel][0], LOW);
       } else {
         PWMInit(motorDirPin[channel][1]);
         PWMStart(channel);
       }
      } else {
       pinMode(motorDirPin[channel][1], OUTPUT);
       digitalWrite(motorDirPin[channel][1], LOW);
       dut[channel] = 0xFF & ((-value) >> 7);
       if(dut[channel] == 0) {
         PWMStop(channel);
         pinMode(motorDirPin[channel][0], OUTPUT);
         digitalWrite(motorDirPin[channel][0], LOW);
       } else {
         PWMInit(motorDirPin[channel][0]);
         PWMStart(channel);
       }
      }
}

void processCommand() {
  i=0;
  if(buf[0] != '!') return;
  
  if(buf[1] == 'G') {
    uint8_t channel = buf[2];
    int16_t value = ((uint16_t)buf[3] << 8) | (uint16_t)buf[4];
    uint8_t checksum = buf[5];

    if((buf[0] + buf[1] + buf[2] + buf[3] + buf[4]) % 255 != checksum) return;

    Serial1.print("# ");
    Serial1.print(channel);
    Serial1.print(" => ");
    Serial1.println(value);

    if(channel >=0 && channel <= 3) {
       set(channel, value);
       lastSetTime = millis();
    }

  }
}

void loop() {

    
  char c;
  
  while(Serial1.available()) {
    if(i > 255) {i=0; Serial1.println("# overrun"); }
    
    c = Serial1.read();
  
    /*Serial1.print("[");
    Serial1.print((int)c);
    Serial1.print("] ");*/
    if(c == '\r') {
      buf[i] = '\0';
      processCommand();
    } else {
      buf[i] = c;
      i++;
    }
  }
  
  if(millis() - lastSetTime > 1000) {
    for(int j=0;j<4;j++) {
      set(j, 0);
    } 
  }
}
