## I2C slave

Implementing I2C slave on ATtiny88.
Basic considerations and procedure.

### credits and further information

- see nxp I&sup2;C specifications and user manual (UM10204.pdf)[http://www.nxp.com/documents/user_manual/UM10204.pdf]
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
