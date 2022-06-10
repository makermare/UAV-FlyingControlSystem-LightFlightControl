#include "include.h"

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
  while (1)
  {
		
  }
}
#endif
//=======================================================================================


//=======================================================================================
u8 Init_Finish = 0;
int main(void)
{
	Init_Finish = All_Init();		
	while(1)
	{
		Duty_Loop(); 
	}
}


