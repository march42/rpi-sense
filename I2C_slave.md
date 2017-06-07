## I2C slave

Implementing I2C slave on ATtiny88.
Basic considerations and procedure.

### credits and further information

- see nxp I&sup2;C specifications and user manual [UM10204.pdf](http://www.nxp.com/documents/user_manual/UM10204.pdf)
- see _atmel-2565-using-the-twi-module-as-i2c-slave_applicationnote_avr311.pdf_ from Atmel for basic information

### good to know

- SCL low period is streched While TWINT flag is set.
- TWINT flag is not automatically cleared by hardware.
- Clearing the TWINT flag `TWCR |= (1 << TWINT);` immediatly starts the TWI operation.
- TWAR, TWSR and TWDR registers are not accessible while TWI in operation.
- Setting TWSTO flag `TWCR |= (1 << TWSTO);` in slave mode releases lines and resets the TWI module to unaddressed state.
- CPU clock must be at least 16 times higher than I2C bus clock frequency. (>1.6MHz for 100kHz bus, >6.4MHz for 400kHz bus)

### I2C bus operation sequence

#### I2C slave receiver - receiving data from master

| **START** | **SLA+W** | ACK | **register address** | ACK | **DATA** | ACK | **DATA** | ACK | **STOP** |

1. MASTER generates START condition, SLAVE listens ~~(TWSR==TW_START)~~
2. MASTER sends SLA+W, SLAVE ACKnowledges (TWSR==TW_SR_SLA_ACK)
3. MASTER sends register address, SLAVE ACKnowledges (TWSR==TW_SR_DATA_ACK)
4. MASTER sends DATA, SLAVE ACKnowledges (TWSR==TW_SR_DATA_ACK)
5. more of 4. till all data is transferred and MASTER sends NACK (TWSR==TW_SR_DATA_NACK)
6. MASTER generates STOP condition, SLAVE finishes (TWSR==TW_SR_STOP)

#### I2C slave transmitter - transmitting data to master (combined WRITE+READ)

| **START** | **SLA+W** | ACK | **register address** | **REP START** | **SLA+R** | ACK | DATA | **ACK** | DATA | **NACK** | **STOP** |

1. MASTER generates START condition, SLAVE listens ~~(TWSR==TW_START)~~
2. MASTER sends SLA+W, SLAVE ACKnowledges (TWSR==TW_SR_SLA_ACK)
3. MASTER sends register address, SLAVE ACKnowledges (TWSR==TW_SR_DATA_ACK)

4. MASTER generates REPEATED START condition, SLAVE listens (TWSR==TW_SR_STOP)

5. MASTER sends SLA+R, SLAVE ACKnowledges (TWSR==TW_ST_SLA_ACK)
6. SLAVE sends DATA, MASTER ACKnowledges (TWSR==TW_ST_DATA_ACK)
7. more of 6. till all data is transferred and MASTER/SLAVE sends NACK (TWSR== ? TW_ST_DATA_NACK ? TW_ST_LAST_DATA)
8. MASTER generates STOP condition, SLAVE finishes (TWSR== ? TW_SR_STOP ? TW_ST_LAST_DATA)

#### I2C slave transmitter - transmitting data to master (READ only, without specific register address)

| **START** | **SLA+R** | ACK | DATA | **ACK** | DATA | **NACK** | **STOP** |

1. MASTER generates START condition, SLAVE listens ~~(TWSR==TW_START)~~
2. MASTER sends SLA+R, SLAVE ACKnowledges (TWSR==TW_ST_SLA_ACK)
3. SLAVE sends DATA, MASTER ACKnowledges (TWSR==TW_ST_DATA_ACK)
4. more of 3. till all data is transferred and MASTER/SLAVE sends NACK (TWSR== ? TW_ST_DATA_NACK ? TW_ST_LAST_DATA)
5. MASTER generates STOP condition, SLAVE finishes (TWSR== ? TW_SR_STOP ? TW_ST_LAST_DATA)

### configuration and building

#### compile time variables for gnu make

| **variable** | **explanation, notes** |
|:---:|:--- |
| DEBUG=1 | enable DEBUG<br/>set compiler USE_REGWRITE, USE_LEDWRITE |
| _DEBUG=_ | disable DEBUG<br/>set compiler NDEBUG, USE_LEDREAD |
| USESLEEP=1 | set compiler USE_SLEEP |
| DISABLE_EXTRAS=1 | disable all extra code and unset compiler DEFINEs |
| _DISABLE_EXTRAS=_ | enable USE_SLEEP and USE_LEDREAD |
| I2C_PAGES=n | set compiler I2C_PAGES=n to enable n register pages |
| WRITE=1 | set compiler USE_REGWRITE, USE_LEDWRITE |

```sh
make all [DEBUG=1] [USESLEEP=1] [DISABLE_EXTRAS=1]
```

#### compile time DEFINEs for avr-gcc

| **DEFINE** | **value** | **explanation, notes** |
|:--- |:---:|:--- |
| I2C_PAGES | 1 _default_ | single page data<br/>**for ATtiny88 with 512Byte SRAM 2 pages is the absolute maximum** |
| I2C_PAGES | 1,2,4,8 | number of register pages<br/>writing 0xAn to REG_WAI changes to page n, WAI='s'+n for all pages [s, t, u,v, w,x,y,z] |
| _NDEBUG_ | _set_ | disable debugging code and optimizations |
| NDEBUG | _unset_ | enable debugging code and optimizations<br/>enable reading of CPU registers |
| USE_REGWRITE | _set_ | enable reading and writing of CPU registers |
| USE_LEDREAD | _set_ | enable code to read LED2472G config and status |
| USE_LEDWRITE | _set_ | enable code to read/write LED2472G config and status |
| I2C_VALIDATE_ADDRESS | _set_ | enable address checking on I2C read/write |
| TWI_VECTOR_S | _set_ | enable C code TWI_vect in rpi-sense-twi.c |

```sh
CDEFINES="[-DI2C_PAGES=2] [-DUSE_LEDWRITE]" make all
```
