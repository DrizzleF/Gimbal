#include "stm32f10x.h"

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
	uint32_t saved_load = SysTick->LOAD;
	uint32_t saved_ctrl = SysTick->CTRL;

	SysTick->LOAD = 72 * xus;
	SysTick->VAL  = 0x00;
	SysTick->CTRL = 0x00000005;              // HCLK, 使能, 无中断
	while (!(SysTick->CTRL & 0x00010000));   // 等待计数到0

	SysTick->LOAD = saved_load;
	SysTick->VAL  = 0x00;
	SysTick->CTRL = saved_ctrl;              // 恢复原始状态（含中断使能）
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 
