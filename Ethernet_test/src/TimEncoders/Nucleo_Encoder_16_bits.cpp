#include "Nucleo_Encoder_16_bits.h"

int32_t Soft_32_Counter_TIM1, Soft_32_Counter_TIM2, Soft_32_Counter_TIM3, Soft_32_Counter_TIM4, Soft_32_Counter_TIM5;

void Overflow_Routine_TIM1()
{
    if(TIM1->SR & 0x0001)
    {
        // printf("Overflow Routine");
        TIM1->SR &= 0xfffe;
        if(!(TIM1->CR1&TIM_CR1_DIR))
            Soft_32_Counter_TIM1 += 0xffff;
        else
            Soft_32_Counter_TIM1 -= 0xffff;
    }
}

void Overflow_Routine_TIM2()
{
    if(TIM2->SR & 0x0001)
    {
        // printf("Overflow Routine");
        TIM2->SR &= 0xfffe;
        if(!(TIM2->CR1&TIM_CR1_DIR))
            Soft_32_Counter_TIM2 += 0xffff;
        else
            Soft_32_Counter_TIM2 -= 0xffff;
    }
}

void Overflow_Routine_TIM3()
{
    if(TIM3->SR & 0x0001)
    {
        // printf("Overflow Routine");
        TIM3->SR &= 0xfffe;
        if(!(TIM3->CR1&TIM_CR1_DIR))
            Soft_32_Counter_TIM3 += 0xffff;
        else
            Soft_32_Counter_TIM3 -= 0xffff;
    }
}
void Overflow_Routine_TIM4()
{
    if(TIM4->SR & 0x0001)
    {
        // printf("Overflow Routine");
        TIM4->SR &= 0xfffe;
        if(!(TIM4->CR1&TIM_CR1_DIR))
            Soft_32_Counter_TIM4 += 0xffff;
        else
            Soft_32_Counter_TIM4 -= 0xffff;
    }
}
void Overflow_Routine_TIM5()
{
    if(TIM5->SR & 0x0001)
    {
        // printf("Overflow Routine");
        TIM5->SR &= 0xfffe;
        if(!(TIM5->CR1&TIM_CR1_DIR))
            Soft_32_Counter_TIM5 += 0xffff;
        else
            Soft_32_Counter_TIM5 -= 0xffff;
    }
}

namespace mbed
{   

    Nucleo_Encoder_16_bits::Nucleo_Encoder_16_bits(TIM_TypeDef * _TIM)
    {
        TIM = _TIM;
        // Initialisation of the TIM module as an encoder counter
        EncoderInit(&encoder, &timer, _TIM, 0xffff, TIM_ENCODERMODE_TI12);

        // Update (aka over- and underflow) interrupt enabled
        TIM->DIER |= 0x0001;
        // The initialisation process generates an update interrupt, so we'll have to clear the update flag before anything else
        TIM->SR &= 0xfffe;

        // Setting the ISR for the corresponding interrupt vector
        switch((uint32_t)TIM)
        {
            case TIM1_BASE :
            NVIC_SetVector(TIM1_UP_TIM10_IRQn, (uint32_t)&Overflow_Routine_TIM1);
            NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
            Soft_32_Counter_TIM1 = 0;
            break;

            case TIM2_BASE :
            NVIC_SetVector(TIM2_IRQn, (uint32_t)&Overflow_Routine_TIM2);
            NVIC_EnableIRQ(TIM2_IRQn);
            Soft_32_Counter_TIM2 = 0;
            break;
            
            case TIM3_BASE :
            NVIC_SetVector(TIM3_IRQn, (uint32_t)&Overflow_Routine_TIM3);
            NVIC_EnableIRQ(TIM3_IRQn);
            Soft_32_Counter_TIM3 = 0;
            break;
            
            case TIM4_BASE :
            NVIC_SetVector(TIM4_IRQn, (uint32_t)&Overflow_Routine_TIM4);
            NVIC_EnableIRQ(TIM4_IRQn);
            Soft_32_Counter_TIM4 = 0;
            break;
            
            case TIM5_BASE :
            NVIC_SetVector(TIM5_IRQn, (uint32_t)&Overflow_Routine_TIM5);
            NVIC_EnableIRQ(TIM5_IRQn);
            Soft_32_Counter_TIM5 = 0;
            break;
            
            default :
            
            break;
        }
        
    }
    Nucleo_Encoder_16_bits::Nucleo_Encoder_16_bits(TIM_TypeDef * _TIM, uint32_t _maxcount, uint32_t _encmode)
    {
        TIM = _TIM;
        // Initialisation of the TIM module as an encoder counter
        EncoderInit(&encoder, &timer, _TIM, _maxcount, _encmode);

        // Update (aka over- and underflow) interrupt enabled
        TIM->DIER |= 0x0001;
        // The initialisation process generates an update interrupt, so we'll have to clear the update flag before anything else
        TIM->SR &= 0xfffe;

        // Setting the ISR for the corresponding interrupt vector
        switch((uint32_t)TIM)
        {
            case TIM1_BASE :
            NVIC_SetVector(TIM1_UP_TIM10_IRQn, (uint32_t)&Overflow_Routine_TIM1);
            NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
            Soft_32_Counter_TIM1 = 0;
            break;

            case TIM2_BASE :
            NVIC_SetVector(TIM2_IRQn, (uint32_t)&Overflow_Routine_TIM2);
            NVIC_EnableIRQ(TIM2_IRQn);
            Soft_32_Counter_TIM2 = 0;
            break;
            
            case TIM3_BASE :
            NVIC_SetVector(TIM3_IRQn, (uint32_t)&Overflow_Routine_TIM3);
            NVIC_EnableIRQ(TIM3_IRQn);
            Soft_32_Counter_TIM3 = 0;
            break;
            
            case TIM4_BASE :
            NVIC_SetVector(TIM4_IRQn, (uint32_t)&Overflow_Routine_TIM4);
            NVIC_EnableIRQ(TIM4_IRQn);
            Soft_32_Counter_TIM4 = 0;
            break;
            
            case TIM5_BASE :
            NVIC_SetVector(TIM5_IRQn, (uint32_t)&Overflow_Routine_TIM5);
            NVIC_EnableIRQ(TIM5_IRQn);
            Soft_32_Counter_TIM5 = 0;
            break;
            
            default :
            
            break;
        }
        
    }
    
    Nucleo_Encoder_16_bits::Nucleo_Encoder_16_bits(TIM_Encoder_InitTypeDef * _encoder, TIM_HandleTypeDef * _timer, TIM_TypeDef * _TIM, uint32_t _maxcount, uint32_t _encmode)
    {
        timer = *_timer;
        encoder = *_encoder;
        TIM = _TIM;
        // Initialisation of the TIM module as an encoder counter
        EncoderInit(&encoder, &timer, _TIM, _maxcount, _encmode);

        // Update (aka over- and underflow) interrupt enabled
        TIM->DIER |= 0x0001;
        // The initialisation process generates an update interrupt, so we'll have to clear the update flag before anything else
        TIM->SR &= 0xfffe;

        // Setting the ISR for the corresponding interrupt vector
        switch((uint32_t)TIM)
        {
            case TIM1_BASE :
            NVIC_SetVector(TIM1_UP_TIM10_IRQn, (uint32_t)&Overflow_Routine_TIM1);
            NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
            Soft_32_Counter_TIM1 = 0;
            break;

            case TIM2_BASE :
            NVIC_SetVector(TIM2_IRQn, (uint32_t)&Overflow_Routine_TIM2);
            NVIC_EnableIRQ(TIM2_IRQn);
            Soft_32_Counter_TIM2 = 0;
            break;
            
            case TIM3_BASE :
            NVIC_SetVector(TIM3_IRQn, (uint32_t)&Overflow_Routine_TIM3);
            NVIC_EnableIRQ(TIM3_IRQn);
            Soft_32_Counter_TIM3 = 0;
            break;
            
            case TIM4_BASE :
            NVIC_SetVector(TIM4_IRQn, (uint32_t)&Overflow_Routine_TIM4);
            NVIC_EnableIRQ(TIM4_IRQn);
            Soft_32_Counter_TIM4 = 0;
            break;
            
            case TIM5_BASE :
            NVIC_SetVector(TIM5_IRQn, (uint32_t)&Overflow_Routine_TIM5);
            NVIC_EnableIRQ(TIM5_IRQn);
            Soft_32_Counter_TIM5 = 0;
            break;
            
            default :
            
            break;
        }

    }

    
    int32_t Nucleo_Encoder_16_bits::GetCounter()
    {
        uint16_t count = TIM->CNT;
        switch((uint32_t)TIM)
        {
            case TIM1_BASE :
            return (int32_t)count + Soft_32_Counter_TIM1;
            break;

            case TIM2_BASE :
            return (int32_t)count + Soft_32_Counter_TIM2;
            break;
            
            case TIM3_BASE :
            return (int32_t)count + Soft_32_Counter_TIM3;
            break;
            
            case TIM4_BASE :
            return (int32_t)count + Soft_32_Counter_TIM4;
            break;
            
            case TIM5_BASE :
            return (int32_t)count + Soft_32_Counter_TIM5;
            break;
        }
        
        return (int32_t)count;
    } 
    
    TIM_HandleTypeDef* Nucleo_Encoder_16_bits::GetTimer()
    {
        return &timer;    
    }

    
    

    
    
}