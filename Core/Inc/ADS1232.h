void Timer_Event();// timer interrupt in which data is being read
void Start_read(TIM_HandleTypeDef *htim_ptr, TIM_TypeDef *TIM_ads);//the beginning of reading this ADC
//                            DOUT_PORT,             SCLK_PORT,              PDWN_PORT,        DOUT_PIN,         SCLK_PIN,           PDWN_PIN,
void set_ADS_pins(GPIO_TypeDef *in_port,GPIO_TypeDef *out_port,GPIO_TypeDef *pdwn_port, uint16_t in_pin, uint16_t out_pin, uint16_t pdwdn_pin, void(*f)(int)); //setting pins for ADC operation
