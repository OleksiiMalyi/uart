#ifndef MY_GPIO_INIT_H
#define MY_GPIO_INIT_H

typedef struct
{
  //uint32_t pin;        
	uint32_t mode;
	uint32_t out;  
	uint32_t pull;       
	uint32_t speed;      
	uint32_t alternate;
}GPIO_init_data;

typedef struct
{
  volatile uint32_t moder;        /*!< GPIO port mode register,               Address offset: 0x00      */
  volatile uint32_t otyper;       /*!< GPIO port output type register,        Address offset: 0x04      */
  volatile uint32_t ospeedr;      /*!< GPIO port output speed register,       Address offset: 0x08      */
  volatile uint32_t pupdr;        /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  volatile uint32_t idr;          /*!< GPIO port input data register,         Address offset: 0x10      */
  volatile uint32_t odr;          /*!< GPIO port output data register,        Address offset: 0x14      */
  volatile uint32_t bsrr;         /*!< GPIO port bit set/reset register,      Address offset: 0x1A */
  volatile uint32_t lckr;         /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  volatile uint32_t afr[2];       /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
  volatile uint32_t brr;          /*!< GPIO bit reset register,               Address offset: 0x28 */
} GPIO_port;

#define PERIPH_BASE           ((uint32_t)0x40000000U)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x08000000U)

#define GPIOA_BASE           (AHB2PERIPH_BASE + 0x00000000U)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x00000400U)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x00000800U)
#define GPIOD_BASE            (AHB2PERIPH_BASE + 0x00000C00U)
#define GPIOE_BASE            (AHB2PERIPH_BASE + 0x00001000U)
#define GPIOF_BASE            (AHB2PERIPH_BASE + 0x00001400U)

#define GPIO_A               ((GPIO_port *) (0x48000000U))
#define GPIO_B               ((GPIO_port *) GPIOB_BASE)
#define GPIO_C               ((GPIO_port *) GPIOC_BASE)
#define GPIO_D               ((GPIO_port *) GPIOD_BASE)
#define GPIO_E               ((GPIO_port *) GPIOE_BASE)
#define GPIO_F               ((GPIO_port *) GPIOF_BASE)


#define DIG_0 ((uint32_t)0)
#define DIG_1 ((uint32_t)1U)
#define DIG_2 ((uint32_t)2U)
#define DIG_3 ((uint32_t)3U)
#define DIG_4 ((uint32_t)4U)
#define DIG_5 ((uint32_t)5U)
#define DIG_6 ((uint32_t)6U) 
#define DIG_7 ((uint32_t)7U)
#define DIG_8 ((uint32_t)8U) 
#define DIG_9 ((uint32_t)9U)
#define DIG_10 ((uint32_t)10U)
#define DIG_11 ((uint32_t)11U) 
#define DIG_12 ((uint32_t)12U) 
#define DIG_13 ((uint32_t)13U)
#define DIG_14 ((uint32_t)14U) 
#define DIG_15 ((uint32_t)15U)
#define DIG_16 ((uint32_t)16U)
#define DIG_17 ((uint32_t)17U)
#define DIG_18 ((uint32_t)18U)
#define DIG_19 ((uint32_t)19U)
#define DIG_20 ((uint32_t)20U)
#define DIG_21 ((uint32_t)21U)
#define DIG_22 ((uint32_t)22U) 
#define DIG_23 ((uint32_t)23U)
#define DIG_24 ((uint32_t)24U) 
#define DIG_25 ((uint32_t)25U)
#define DIG_26 ((uint32_t)26U)
#define DIG_27 ((uint32_t)27U) 
#define DIG_28 ((uint32_t)28U) 
#define DIG_29 ((uint32_t)29U)
#define DIG_30 ((uint32_t)30U) 
#define DIG_31 ((uint32_t)31U) 

 
#define PIN_0 ((uint32_t)0U)
#define PIN_1 ((uint32_t)2U)
#define PIN_2 ((uint32_t)4U)
#define PIN_3 ((uint32_t)6U)
#define PIN_4 ((uint32_t)8U)
#define PIN_5 ((uint32_t)10U)
#define PIN_6 ((uint32_t)12U) 
#define PIN_7 ((uint32_t)14U)
#define PIN_8 ((uint32_t)16U) 
#define PIN_9 ((uint32_t)18U)
#define PIN_10 ((uint32_t)20U)
#define PIN_11 ((uint32_t)22U) 
#define PIN_12 ((uint32_t)24U) 
#define PIN_13 ((uint32_t)26U)
#define PIN_14 ((uint32_t)28U) 
#define PIN_15 ((uint32_t)30U)

#define INPUT_MODE 0x00U 	// to set input mode (GPIO)
#define OUTPUT_MODE 0x01U	// to set output mode (GPIO)
#define ALTERNATE_MODE 0x02U	// to set alternate function mode (GPIO)
#define ANALOG_MODE 0x03U	// to set analog mode (GPIO)

#define LOW_SPEED 0x00U
#define MEDIUM_SPEED 0x01U
#define HIGH_SPEED 0x03U
#define MY_SPEED 0x02U

#define PULL_UP 0x01U
#define PULL_DOWN 0x02U

#define HIGH (uint32_t) 1U // TRUE for any single bit
#define LOW (uint32_t) 0U	// FALSE for any single bit
#endif //MY_GPIO_INIT_H

#ifndef SET_BIT // for all of cases 
	#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

	#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

	#define READ_BIT(REG, BIT)    ((REG) & (BIT))

	#define CLEAR_REG(REG)        ((REG) = (0x0))

	#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
	
	#define READ_REG(REG)         ((REG))

	#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

	#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL))) 
#endif // end of this definition

// prototypes of functions
void MY_GPIO_init(void);
void my_GPIO_init_port(GPIO_port *, GPIO_init_data*);

void my_send_pack(GPIO_port *, uint32_t, uint32_t);
void my_delay(uint32_t);



