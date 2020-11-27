# MS51_Breathing_LED
 MS51_Breathing_LED

update @ 2020/11/27

1. use P12 as PWM (32K) , to demo breahthing light

2. use 1 timer , to detect timer loop : loop_1ms , loop_10ms , loop_100ms , loop_1000ms , loop_5000ms

3. use P11 , P13 , P14 as GPIO , to detect each timer loop

{

void loop_1000ms(void)

{	

	if (is_flag_set(flag_1000ms))
	
	{		
	
		set_flag(flag_1000ms,Disable);

		// toggle test
		
		P13 = ~P13;
		
	}
	
}

}

{

void loop_5000ms(void)

{	
	if (is_flag_set(flag_5000ms))
	
	{		
	
		set_flag(flag_5000ms,Disable);

		// toggle test
		
		P14 = ~P14;
			
	}
	
}

}


below is screen capture

![image](https://github.com/released/MS51_Breathing_LED/blob/main/1000ms_5000ms.jpg)

![image](https://github.com/released/MS51_Breathing_LED/blob/main/PWM_swap.jpg)


