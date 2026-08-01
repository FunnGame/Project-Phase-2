#include "stm32f10x.h"
#include "../inc/drv_i2c.h"

/* Private Macro */

#define DRV_I2C_TIMEOUT        (100000U)

#define I2C1_GPIO_PORT         GPIOB
#define I2C2_GPIO_PORT         GPIOB

#define I2C1_SCL_PIN           6U
#define I2C1_SDA_PIN           7U

#define I2C2_SCL_PIN           10U
#define I2C2_SDA_PIN           11U

#define I2C_APB1_CLOCK         36000000U


/*  Private Function Prototypes */

static I2C_TypeDef* I2C_GetInstance(I2C_t i2c);

static void I2C_EnableClock(I2C_t i2c);

static void I2C_GPIO_Init(I2C_t i2c);

static Status_t I2C_WaitFlag(I2C_TypeDef *I2Cx,
                             uint32_t flag);

static Status_t I2C_CheckError(I2C_TypeDef *I2Cx);

static Status_t I2C_WaitBusyClear(I2C_TypeDef *I2Cx);

static Status_t I2C_Start(I2C_TypeDef *I2Cx);

static Status_t I2C_SendAddress(I2C_TypeDef *I2Cx,
                                uint8_t address,
                                I2C_Direction_t direction);

static Status_t I2C_WriteByte(I2C_TypeDef *I2Cx,
                              uint8_t data);

static Status_t I2C_ReadByte(I2C_TypeDef *I2Cx,
                             uint8_t *data,
                             uint8_t ack);

static void I2C_Stop(I2C_TypeDef *I2Cx);


/*  Private Functions  */

static I2C_TypeDef* I2C_GetInstance(I2C_t i2c)
{
    switch(i2c)
    {
        case I2C_1:
            return I2C1;

        case I2C_2:
            return I2C2;

        default:
            return (I2C_TypeDef *)0;
    }
}


static void I2C_EnableClock(I2C_t i2c)
{
    switch(i2c)
    {
        case I2C_1:

            /* GPIOB clock */
            RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

            /* I2C1 clock */
            RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

            break;


        case I2C_2:

            /* GPIOB clock */
            RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

            /* I2C2 clock */
            RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

            break;


        default:
            break;
    }
}


static void I2C_GPIO_Init(I2C_t i2c)
{
  

    if(i2c == I2C_1)
    {
        GPIOB->CRL &= ~((0xFU << 24U) |
                        (0xFU << 28U));

        GPIOB->CRL |= ((0xFU << 24U) |
                       (0xFU << 28U));
    }
    else if(i2c == I2C_2)
    {
        GPIOB->CRH &= ~((0xFU << 8U) |
                        (0xFU << 12U));

        GPIOB->CRH |= ((0xFU << 8U) |
                       (0xFU << 12U));
    }
}


static Status_t I2C_WaitFlag(I2C_TypeDef *I2Cx,
                             uint32_t flag)
{
    uint32_t timeout = DRV_I2C_TIMEOUT;

    while((I2Cx->SR1 & flag) == 0U)
    {
        if(I2C_CheckError(I2Cx) != STATUS_OK)
        {
            return STATUS_ERROR;
        }

        timeout--;

        if(timeout == 0U)
        {
            return STATUS_TIMEOUT;
        }
    }

    return STATUS_OK;
}


static Status_t I2C_CheckError(I2C_TypeDef *I2Cx)
{
    uint32_t error = I2Cx->SR1;

    if((error & I2C_SR1_BERR) != 0U)
    {
        I2Cx->SR1 &= ~I2C_SR1_BERR;
        return STATUS_ERROR;
    }

    if((error & I2C_SR1_ARLO) != 0U)
    {
        I2Cx->SR1 &= ~I2C_SR1_ARLO;
        return STATUS_ERROR;
    }

    if((error & I2C_SR1_AF) != 0U)
    {
        I2Cx->SR1 &= ~I2C_SR1_AF;
        return STATUS_ERROR;
    }

    if((error & I2C_SR1_OVR) != 0U)
    {
        I2Cx->SR1 &= ~I2C_SR1_OVR;
        return STATUS_ERROR;
    }

    return STATUS_OK;
}


static Status_t I2C_WaitBusyClear(I2C_TypeDef *I2Cx)
{
    uint32_t timeout = DRV_I2C_TIMEOUT;

    while((I2Cx->SR2 & I2C_SR2_BUSY) != 0U)
    {
        timeout--;

        if(timeout == 0U)
        {
            return STATUS_TIMEOUT;
        }
    }

    return STATUS_OK;
}


static Status_t I2C_Start(I2C_TypeDef *I2Cx)
{
    I2Cx->CR1 |= I2C_CR1_START;

    return I2C_WaitFlag(I2Cx, I2C_SR1_SB);
}


static Status_t I2C_SendAddress(I2C_TypeDef *I2Cx,
                                uint8_t address,
                                I2C_Direction_t direction)
{
    uint8_t addressByte;

    addressByte = (uint8_t)((address << 1U) |
                            (uint8_t)direction);

    I2Cx->DR = addressByte;

    if(I2C_WaitFlag(I2Cx, I2C_SR1_ADDR) != STATUS_OK)
    {
        return STATUS_ERROR;
    }

    /* ADDR flag is cleared by reading SR1 followed by SR2 */
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;

    return STATUS_OK;
}


static Status_t I2C_WriteByte(I2C_TypeDef *I2Cx,
                              uint8_t data)
{
    I2Cx->DR = data;

    if(I2C_WaitFlag(I2Cx, I2C_SR1_TXE) != STATUS_OK)
    {
        return STATUS_ERROR;
    }

    return STATUS_OK;
}


static Status_t I2C_ReadByte(I2C_TypeDef *I2Cx,
                             uint8_t *data,
                             uint8_t ack)
{
    if(data == (uint8_t *)0)
    {
        return STATUS_INVALID_PARAM;
    }

    if(ack != 0U)
    {
        I2Cx->CR1 |= I2C_CR1_ACK;
    }
    else
    {
        I2Cx->CR1 &= ~I2C_CR1_ACK;
    }

    if(I2C_WaitFlag(I2Cx, I2C_SR1_RXNE) != STATUS_OK)
    {
        return STATUS_ERROR;
    }

    *data = (uint8_t)I2Cx->DR;

    return STATUS_OK;
}


static void I2C_Stop(I2C_TypeDef *I2Cx)
{
    I2Cx->CR1 |= I2C_CR1_STOP;
}


/*  Public Functions */

Status_t DRV_I2C_Init(I2C_t i2c,
                      I2C_Speed_t speed)
{
    I2C_TypeDef *I2Cx;

    I2Cx = I2C_GetInstance(i2c);

    if(I2Cx == (I2C_TypeDef *)0)
    {
        return STATUS_INVALID_PARAM;
    }

    if((speed != I2C_SPEED_100KHZ) &&
       (speed != I2C_SPEED_400KHZ))
    {
        return STATUS_INVALID_PARAM;
    }

    I2C_EnableClock(i2c);

    I2C_GPIO_Init(i2c);

    /*
     * Disable I2C before configuration.
     */
    I2Cx->CR1 &= ~I2C_CR1_PE;

    /*
     * APB1 clock = 36 MHz
     */
    I2Cx->CR2 &= ~I2C_CR2_FREQ;
    I2Cx->CR2 |= 36U;

    /*
     * Configure clock speed.
     */
    I2Cx->CCR &= ~I2C_CCR_CCR;

    if(speed == I2C_SPEED_100KHZ)
    {
       
        I2Cx->CCR |= 180U;
    }
    else
    {
        
        I2Cx->CCR |= 30U;
    }

    I2Cx->TRISE &= ~I2C_TRISE_TRISE;

    if(speed == I2C_SPEED_100KHZ)
    {
        I2Cx->TRISE |= 37U;
    }
    else
    {
        I2Cx->TRISE |= 11U;
    }

    /*
     * Enable ACK.
     */
    I2Cx->CR1 |= I2C_CR1_ACK;

    /*
     * Enable I2C peripheral.
     */
    I2Cx->CR1 |= I2C_CR1_PE;

    return STATUS_OK;
}


Status_t DRV_I2C_Write(I2C_t i2c,
                       uint8_t address,
                       uint16_t reg,
                       const uint8_t *data,
                       uint16_t length)
{
    I2C_TypeDef *I2Cx;
    uint16_t i;

    if((data == (const uint8_t *)0) &&
       (length > 0U))
    {
        return STATUS_INVALID_PARAM;
    }

    I2Cx = I2C_GetInstance(i2c);

    if(I2Cx == (I2C_TypeDef *)0)
    {
        return STATUS_INVALID_PARAM;
    }

    if(I2C_WaitBusyClear(I2Cx) != STATUS_OK)
    {
        return STATUS_TIMEOUT;
    }

    if(I2C_Start(I2Cx) != STATUS_OK)
    {
        I2C_Stop(I2Cx);
        return STATUS_ERROR;
    }

    if(I2C_SendAddress(I2Cx,
                       address,
                       I2C_WRITE) != STATUS_OK)
    {
        I2C_Stop(I2Cx);
        return STATUS_ERROR;
    }
/* Register address is 16-bit, MSB first */
if(I2C_WriteByte(I2Cx, (uint8_t)(reg >> 8U)) != STATUS_OK)
{
    I2C_Stop(I2Cx);
    return STATUS_ERROR;
}

if(I2C_WriteByte(I2Cx, (uint8_t)(reg & 0xFFU)) != STATUS_OK)
{
    I2C_Stop(I2Cx);
    return STATUS_ERROR;
}
    
    for(i = 0U; i < length; i++)
    {
        if(I2C_WriteByte(I2Cx, data[i]) != STATUS_OK)
        {
            I2C_Stop(I2Cx);
            return STATUS_ERROR;
        }
    }

    /*
     * Wait until transfer is completely finished.
     */
    if(I2C_WaitFlag(I2Cx, I2C_SR1_BTF) != STATUS_OK)
    {
        I2C_Stop(I2Cx);
        return STATUS_TIMEOUT;
    }

    I2C_Stop(I2Cx);

    return STATUS_OK;
}


Status_t DRV_I2C_Read(I2C_t i2c,
                      uint8_t address,
                      uint16_t reg,
                      uint8_t *data,
                      uint16_t length)
{
    I2C_TypeDef *I2Cx;
    uint16_t i;

    if((data == (uint8_t *)0) ||
       (length == 0U))
    {
        return STATUS_INVALID_PARAM;
    }

    I2Cx = I2C_GetInstance(i2c);

    if(I2Cx == (I2C_TypeDef *)0)
    {
        return STATUS_INVALID_PARAM;
    }

    if(I2C_WaitBusyClear(I2Cx) != STATUS_OK)
    {
        return STATUS_TIMEOUT;
    }

    /*
     * START
     */
    if(I2C_Start(I2Cx) != STATUS_OK)
    {
        I2C_Stop(I2Cx);
        return STATUS_ERROR;
    }

    /*
     * Slave address + WRITE
     */
    if(I2C_SendAddress(I2Cx,
                       address,
                       I2C_WRITE) != STATUS_OK)
    {
        I2C_Stop(I2Cx);
        return STATUS_ERROR;
    }

    /*
     * Register address
     */
    /* Register address is 16-bit, MSB first */
if(I2C_WriteByte(I2Cx, (uint8_t)(reg >> 8U)) != STATUS_OK)
{
    I2C_Stop(I2Cx);
    return STATUS_ERROR;
}

if(I2C_WriteByte(I2Cx, (uint8_t)(reg & 0xFFU)) != STATUS_OK)
{
    I2C_Stop(I2Cx);
    return STATUS_ERROR;
}

    /*
     * Repeated START
     */
    if(I2C_Start(I2Cx) != STATUS_OK)
    {
        I2C_Stop(I2Cx);
        return STATUS_ERROR;
    }

    /*
     * Slave address + READ
     */
    if(I2C_SendAddress(I2Cx,
                       address,
                       I2C_READ) != STATUS_OK)
    {
        I2C_Stop(I2Cx);
        return STATUS_ERROR;
    }

    for(i = 0U; i < length; i++)
    {
        if(i == (length - 1U))
        {
            /*
             * Last byte -> NACK
             */
            if(I2C_ReadByte(I2Cx,
                            &data[i],
                            0U) != STATUS_OK)
            {
                I2C_Stop(I2Cx);
                return STATUS_ERROR;
            }
        }
        else
        {
            /*
             * More bytes -> ACK
             */
            if(I2C_ReadByte(I2Cx,
                            &data[i],
                            1U) != STATUS_OK)
            {
                I2C_Stop(I2Cx);
                return STATUS_ERROR;
            }
        }
    }

    I2C_Stop(I2Cx);

    /*
     * Restore ACK for next transaction.
     */
    I2Cx->CR1 |= I2C_CR1_ACK;

    return STATUS_OK;
}


Status_t DRV_I2C_Reset(I2C_t i2c)
{
    I2C_TypeDef *I2Cx;

    I2Cx = I2C_GetInstance(i2c);

    if(I2Cx == (I2C_TypeDef *)0)
    {
        return STATUS_INVALID_PARAM;
    }

    /*
     * Disable peripheral.
     */
    I2Cx->CR1 &= ~I2C_CR1_PE;

    /*
     * Software reset.
     */
    I2Cx->CR1 |= I2C_CR1_SWRST;

    /*
     * Release reset.
     */
    I2Cx->CR1 &= ~I2C_CR1_SWRST;

    return STATUS_OK;
}