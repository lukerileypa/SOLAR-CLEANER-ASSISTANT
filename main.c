  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
char* echo;
char* num;
char* studnum = "23835990";
//HAL_UART_Transmit(&huart2,"&_",2,100);
HAL_UART_Transmit(&huart2,studnum,8,100);
//HAL_UART_Transmit(&huart2,"_*",2,100);
HAL_UART_Transmit(&huart2,"\n",1,100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_UART_Recieve(&huart2,(uint8_t*)echo,1,100);
	  if(echo!=num){
		  HAL_UART_Transmit(&huart2,(uint8_t*)echo,1,100);
	  }
	  echo = num;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
