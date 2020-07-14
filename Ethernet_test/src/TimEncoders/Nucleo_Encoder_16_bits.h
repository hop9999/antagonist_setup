#ifndef ENCODER_H
#define ENCODER_H
#include "mbed.h"

void EncoderInit(TIM_Encoder_InitTypeDef * encoder, TIM_HandleTypeDef * timer, TIM_TypeDef * TIMx, uint32_t maxcount, uint32_t encmode);


namespace mbed{
    
    class Nucleo_Encoder_16_bits {
        
        public :
            Nucleo_Encoder_16_bits(TIM_TypeDef * _TIM);
            Nucleo_Encoder_16_bits(TIM_TypeDef * _TIM, uint32_t _maxcount, uint32_t _encmode);
            Nucleo_Encoder_16_bits(TIM_Encoder_InitTypeDef * _encoder, TIM_HandleTypeDef * _timer, TIM_TypeDef * _TIM, uint32_t _maxcount, uint32_t _encmode);
            int32_t GetCounter();
            TIM_HandleTypeDef* GetTimer();
        
        private :
        
        
        protected :
            TIM_Encoder_InitTypeDef encoder;
            TIM_HandleTypeDef timer;
            TIM_TypeDef * TIM;
        
    };

}// namespace mbed

#endif