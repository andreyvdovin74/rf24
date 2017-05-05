/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 
 
 
03/17/2013 : Charles-Henri Hallard (http://hallard.me)
             Modified to use with Arduipi board http://hallard.me/arduipi
                         Changed to use modified bcm2835 library

*/

#include "QRF24Network_config.h"
#include "QRF24.h"
#include "nRF24L01.h"


#include <QDebug>

#if !defined (MINIMAL)

static const char rf24_datarate_e_str_0[] = "1MBPS";
static const char rf24_datarate_e_str_1[] = "2MBPS";
static const char rf24_datarate_e_str_2[] = "250KBPS";
static const char * const rf24_datarate_e_str_P[] = {
    rf24_datarate_e_str_0,
    rf24_datarate_e_str_1,
    rf24_datarate_e_str_2,
};
static const char rf24_model_e_str_0[] = "nRF24L01";
static const char rf24_model_e_str_1[] = "nRF24L01+";
static const char * const rf24_model_e_str_P[] = {
    rf24_model_e_str_0,
    rf24_model_e_str_1,
};
static const char rf24_crclength_e_str_0[] = "Disabled";
static const char rf24_crclength_e_str_1[] = "8 bits";
static const char rf24_crclength_e_str_2[] = "16 bits" ;
static const char * const rf24_crclength_e_str_P[] = {
    rf24_crclength_e_str_0,
    rf24_crclength_e_str_1,
    rf24_crclength_e_str_2,
};
static const char rf24_pa_dbm_e_str_0[] = "PA_MIN";
static const char rf24_pa_dbm_e_str_1[] = "PA_LOW";
static const char rf24_pa_dbm_e_str_2[] = "PA_HIGH";
static const char rf24_pa_dbm_e_str_3[] = "PA_MAX";
static const char * const rf24_pa_dbm_e_str_P[] = {
    rf24_pa_dbm_e_str_0,
    rf24_pa_dbm_e_str_1,
    rf24_pa_dbm_e_str_2,
    rf24_pa_dbm_e_str_3,
};


static const char rf24_csn_e_str_0[] = "CE0 (PI Hardware Driven)";
static const char rf24_csn_e_str_1[] = "CE1 (PI Hardware Driven)";
static const char rf24_csn_e_str_2[] = "CE2 (PI Hardware Driven)";
static const char rf24_csn_e_str_3[] = "Custom GPIO Software Driven";
static const char * const rf24_csn_e_str_P[] = {
   rf24_csn_e_str_0,
   rf24_csn_e_str_1,
   rf24_csn_e_str_2,
   rf24_csn_e_str_3,
};
#endif

QRF24::QRF24(quint8 _cepin, quint8 _cspin, uint32_t _spi_speed, QObject *parent):
    QObject(parent),
    ce_pin(_cepin),
    csn_pin(_cspin),
    spi_speed(_spi_speed),
    wide_band(true),
    p_variant(false),
    payload_size(32),
    ack_payload_available(true),
    dynamic_payloads_enabled(false),
    pipe0_reading_address(0)
{
    debug = false;
}

quint8 QRF24::readRegister(quint8 reg, quint8* buf, quint8 len)
{
    quint8 status;
    quint8 * prx = spi_rxbuff;
    quint8 * ptx = spi_txbuff;
    quint8 size = len + 1; // Add register value to transmit buffer

    *ptx++ = ( R_REGISTER | ( REGISTER_MASK & reg ) );
    while (len--) *ptx++ = NOP ; // Dummy operation, just for reading

    bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);

    status = *prx++; // status is 1st byte of receive buffer

    // decrement before to skip status byte
    while ( --size ) *buf++ = *prx++;
    return status;
}

quint8 QRF24::readRegister(quint8 reg)
{
    quint8 result;
    quint8 * prx = spi_rxbuff;
    quint8 * ptx = spi_txbuff;

    *ptx++ = ( R_REGISTER | ( REGISTER_MASK & reg ) );
    *ptx = NOP ; // Dummy operation, just for reading

    bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, 2);

    result = *++prx;   // result is 2nd byte of receive buffer

    return result;
}



quint8 QRF24::writeRegister(quint8 reg, quint8 value)
{
    quint8 status;
    quint8 * prx = spi_rxbuff;
    quint8 * ptx = spi_txbuff;

    *ptx++ = ( W_REGISTER | ( REGISTER_MASK & reg ) );
    *ptx = value ;

    bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, 2);
    status = *prx++; // status is 1st byte of receive buffer
    //if (debug) qDebug() << QString ("reg=%1,value=%2").arg(reg,0,16).arg(value,0,16);
    return status;
}



quint8 QRF24::writeRegister(quint8 reg, const quint8* buf, quint8 len)
{
    quint8 status;
    quint8 * prx = spi_rxbuff;
    quint8 * ptx = spi_txbuff;
    quint8 size = len + 1; // Add register value to transmit buffer

    *ptx++ = ( W_REGISTER | ( REGISTER_MASK & reg ) );
    while ( len-- )
        *ptx++ = *buf++;

    bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);

    status = *prx; // status is 1st byte of receive buffer

    return status;
}

quint8 QRF24::writePayload(const void* buf, quint8 len)
{
    quint8 status;
    quint8 * prx = spi_rxbuff;
    quint8 * ptx = spi_txbuff;
    quint8 size ;

    const quint8* current = reinterpret_cast<const quint8*>(buf);

    quint8 data_len = qMin(len,payload_size);
    quint8 blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

    size = data_len + blank_len + 1 ; // Add register value to transmit buffer

    //if (debug) qDebug() << QString ("Writing %1 bytes %2 blanks").arg(data_len).arg(blank_len);

    *ptx++ =  W_TX_PAYLOAD;
    while ( data_len-- )
        *ptx++ =  *current++;
    while ( blank_len-- )
        *ptx++ =  0;

    bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);

    status = *prx; // status is 1st byte of receive buffer
    //qDebug() << QByteArray(*ptx,sizeof(ptx)).toHex();

    return status;
}



quint8 QRF24::readPayload(void* buf, quint8 len)
{
    quint8 status;
    quint8 * prx = spi_rxbuff;
    quint8 * ptx = spi_txbuff;
    quint8 size ;

    quint8* current = reinterpret_cast<quint8*>(buf);

    quint8 data_len = qMin(len,payload_size);
    quint8 blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

    size = data_len + blank_len + 1; // Add register value to transmit buffer
     qDebug() << QString("Reading %1 bytes %2 blanks").arg(data_len).arg(blank_len);

    *ptx++ =  R_RX_PAYLOAD;
    while(size--) *ptx++ = NOP;
        //qDebug() << QString("ReadPayload %1").arg(*ptx);

    // Size has been lost during while, re affect
    size = data_len + blank_len + 1; // Add register value to transmit buffer

    bcm2835_spi_transfernb( (char *) spi_txbuff, (char *) spi_rxbuff, size);
    // 1st byte is status
    status = *prx++;

    // Decrement before to skip 1st status byte
    while ( --size ) *current++ = *prx++;
    //qDebug() << QByteArray(*prx,sizeof(prx)).toHex();
    return status;
}
quint8 QRF24::flushRx()
{
    quint8 status;
    status = bcm2835_spi_transfer( FLUSH_RX );
    return status;
}
quint8 QRF24::flushTx()
{
    quint8 status;
    status = bcm2835_spi_transfer( FLUSH_TX );
    return status;
}
quint8 QRF24::getStatus()
{
    quint8 status;
    status = bcm2835_spi_transfer( NOP );
    return status;
}
QString QRF24::status(quint8 status) const
{
    return QString ("STATUS = 0x%1 RX_DR = %2 TX_DS = %3 MAX_RT = %4 RX_P_NO = %5 TX_FULL=%6")
                                 .arg(status,0,16)
                                 .arg(status & _BV(RX_DR)?1:0)
                                 .arg(status & _BV(TX_DS)?1:0)
                                 .arg(status & _BV(MAX_RT)?1:0)
                                 .arg((status >> RX_P_NO) & 0b111)
                                 .arg(status & _BV(TX_FULL)?1:0);
}
QString QRF24::printObserveTx(quint8 value) const
{
    return QString ("Observe_TX=%1, POLS_CNT=%2, ARC_CNT=%3").arg(value).arg(value >> PLOS_CNT & 0b01111).arg(value >> ARC_CNT & 0b1111);
}
QString QRF24::byteRegister(const char* name, quint8 reg, quint8 qty)
{
    QString result;
    QString printfQString;
    char extra_tab = strlen(name) < 8 ? '\t' : 0;
    result.append(printfQString.sprintf("%s\t%c =", name, extra_tab));
    while (qty--)
        result.append(printfQString.sprintf(" 0x%02x",readRegister(reg++)));
    result.append("\n");
    return result;
}



QString QRF24::addressRegister(const char* name, quint8 reg, quint8 qty)
{
    QString printfQString;
    QString result;
    char extra_tab = strlen(name) < 8 ? '\t' : 0;
    result.append(printfQString.sprintf("%s\t%c =",name,extra_tab));

    while (qty--)
    {
        quint8 buffer[5];
        readRegister(reg++,buffer,sizeof buffer);

        result.append(" 0x");
        quint8* bufptr = buffer + sizeof buffer;
        while( --bufptr >= buffer )
            result.append(printfQString.sprintf("%02x",*bufptr));
    }

    result.append("\r\n");
    return result;
}

void QRF24::setChannel(quint8 channel)
{
    // TODO: This method could take advantage of the 'wide_band' calculation
    // done in setChannel() to require certain channel spacing.

    const quint8 max_channel = 127;
    writeRegister(RF_CH,qMin(channel,max_channel));
}

void QRF24::setPayloadSize(quint8 size)
{
    const quint8 max_payload_size = 32;
    payload_size = qMin(size,max_payload_size);
}
quint8 QRF24::getPayloadSize()
{
    return payload_size;
}
void QRF24::getDetails(bool displayDetails)
{
    if (displayDetails)
{
    qDebug() << ("================ SPI Configuration ================  ");

    if (csn_pin < BCM2835_SPI_CS_NONE )
    {
        qDebug() << QString ("CSN Pin = %1").arg(rf24_csn_e_str_P[csn_pin]);
    }
    else
    {
        qDebug() << QString ("CSN Pin = Custom GPIO =%1, (CE1) Software Driven:%2").arg(csn_pin).arg(csn_pin==RPI_V2_GPIO_P1_26);
    }
    qDebug() << QString ("CE Pin = Custom GPIO%1").arg(ce_pin);

    // SPI Bus Speed
    switch (spi_speed)
    {
    case BCM2835_SPI_SPEED_64MHZ : qDebug() << ("Clock Speed = 64 Mhz"); break ;
    case BCM2835_SPI_SPEED_32MHZ : qDebug() << ("Clock Speed = 32 Mhz"); break ;
    case BCM2835_SPI_SPEED_16MHZ : qDebug() << ("Clock Speed = 16 Mhz");	break ;
    case BCM2835_SPI_SPEED_8MHZ  : qDebug() << ("Clock Speed = 8 Mhz"); 	break ;
    case BCM2835_SPI_SPEED_4MHZ  : qDebug() << ("Clock Speed = 4 Mhz"); 	break ;
    case BCM2835_SPI_SPEED_2MHZ  : qDebug() << ("Clock Speed = 2 Mhz"); 	break ;
    case BCM2835_SPI_SPEED_1MHZ  : qDebug() << ("Clock Speed = 1 Mhz"); 	break ;
    case BCM2835_SPI_SPEED_512KHZ: qDebug() << ("Clock Speed = 512 KHz");break ;
    case BCM2835_SPI_SPEED_256KHZ: qDebug() << ("Clock Speed = 256 KHz");break ;
    case BCM2835_SPI_SPEED_128KHZ: qDebug() << ("Clock Speed = 128 KHz");	break ;
    case BCM2835_SPI_SPEED_64KHZ : qDebug() << ("Clock Speed = 64 KHz"); 	break ;
    case BCM2835_SPI_SPEED_32KHZ : qDebug() << ("Clock Speed = 32 KHz"); 	break ;
    case BCM2835_SPI_SPEED_16KHZ : qDebug() << ("Clock Speed = 16 KHz"); 	break ;
    case BCM2835_SPI_SPEED_8KHZ  : qDebug() << ("Clock Speed = 8 KHz"); 	break ;
    default : qDebug() << ("Probably Bad !!!"); break ;
    }
    qDebug() << ("================ NRF Configuration ================\n" );
    qDebug() << QString("%1").arg(status(getStatus()));

    qDebug() << QString ("%1").arg(addressRegister("RX_ADDR_P0-1",RX_ADDR_P0,2));
    qDebug() << QString ("%1").arg(byteRegister("RX_ADDR_P2-5",RX_ADDR_P2,4));
    qDebug() << QString ("%1").arg(addressRegister("TX_ADDR",TX_ADDR));
    qDebug() << QString ("%1").arg(byteRegister("TX_ADDR",TX_ADDR));
    qDebug() << QString ("%1").arg(byteRegister("RX_PW_P0-6",RX_PW_P0,6));
    qDebug() << QString ("%1").arg(byteRegister("EN_AA",EN_AA));
    qDebug() << QString ("%1").arg(byteRegister("RF_CH",RF_CH));
    qDebug() << QString ("%1").arg(byteRegister("RF_SETUP",RF_SETUP));
    qDebug() << QString ("%1").arg(byteRegister("CONFIG",CONFIG));
    qDebug() << QString ("%1").arg(byteRegister("DYNPD/FEATURE",DYNPD,2));
    qDebug() << QString ("Data Rate = %1").arg(rf24_datarate_e_str_P[getDataRate()]);
    qDebug() << QString ("Model = %1").arg(rf24_model_e_str_P[isPVariant()]);
    qDebug() << QString ("CRC Length = %1").arg(rf24_crclength_e_str_P[getCRCLength()]);
    qDebug() << QString ("PA Power = %1").arg(rf24_pa_dbm_e_str_P[getPALevel()]);
    }
}

bool QRF24::begin()
{
    // Init BCM2835 chipset for talking with us
    if (!bcm2835_init())
        return false;

    // Initialise the CE pin of NRF24 (chip enable)
    bcm2835_gpio_fsel(ce_pin, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(ce_pin, LOW);

    // used to drive custom I/O to trigger my logic analyser
    // bcm2835_gpio_fsel(GPIO_CTRL_PIN , BCM2835_GPIO_FSEL_OUTP);

    // start the SPI library:
    // Note the NRF24 wants mode 0, MSB first and default to 1 Mbps
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);

    // Set SPI bus Speed
    bcm2835_spi_setClockSpeed(spi_speed);

    // This initialize the SPI bus with
    // csn pin as chip select (custom or not)
    bcm2835_spi_begin(csn_pin);

    // wait 100ms
    delay(100);

    // Must allow the radio time to settle else configuration bits will not necessarily stick.
    // This is actually only required following power up but some settling time also appears to
    // be required after resets too. For full coverage, we'll always assume the worst.
    // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
    // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
    // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
    delay( 5 ) ;

    // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
    // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
    // sizes must never be used. See documentation for a more complete explanation.
    //printf("write_register(%02X, %02X)\n", SETUP_RETR, (0b0100 << ARD) | (0b1111 << ARC));
    writeRegister(SETUP_RETR,(0b0100 << ARD) | (0b1111 << ARC));

    // Restore our default PA level
    setPALevel( RF24_PA_MAX ) ;

    // Determine if this is a p or non-p RF24 module and then
    // reset our data rate back to default value. This works
    // because a non-P variant won't allow the data rate to
    // be set to 250Kbps.
    if( setDataRate( RF24_250KBPS ) )
    {
        p_variant = true ;
    }

    // Then set the data rate to the slowest (and most reliable) speed supported by all
    // hardware.
    setDataRate( RF24_1MBPS ) ;

    // Initialize CRC and request 2-byte (16bit) CRC
    setCRCLength( RF24_CRC_16 ) ;

    // Disable dynamic payloads, to match dynamic_payloads_enabled setting
    writeRegister(DYNPD,0);

    // Reset current status
    // Notice reset and flush is the last thing we do
    writeRegister(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

    // Set up default configuration.  Callers can always change it later.
    // This channel should be universally safe and not bleed over into adjacent
    // spectrum.
    setChannel(76);

    // Flush buffers
    flushRx();
    flushTx();

    return true;
}
void QRF24::startListening()
{
    writeRegister(CONFIG, readRegister(CONFIG) | _BV(PWR_UP) | _BV(PRIM_RX));
    writeRegister(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

    // Restore the pipe0 adddress, if exists
//    if (pipe0_reading_address)
    //writeRegister(RX_ADDR_P0,)
    writeRegister(RX_ADDR_P0, reinterpret_cast<const quint8*>(&mypipe0), 5);
    qDebug() << QString ("pipe0_reading_address 0x%1").arg(mypipe0,0,16);
    // Flush buffers
    flushRx();
    flushTx();

    // Go!
    bcm2835_gpio_write(ce_pin, HIGH);
    // wait for the radio to come up (130us actually only needed)
    delayMicroseconds(130);
}
void QRF24::stopListening()
{
    bcm2835_gpio_write(ce_pin, LOW);
    flushTx();
    flushRx();
}
void QRF24::powerDown()
{
    writeRegister(CONFIG,readRegister(CONFIG) & ~_BV(PWR_UP));
}
void QRF24::powerUp()
{
    writeRegister(CONFIG,readRegister(CONFIG) | _BV(PWR_UP));
}
/******************************************************************/

bool QRF24::write( const void* buf, quint8 len )
{
    bool result = false;

    // Begin the write
    startWrite(buf,len);

    // ------------
    // At this point we could return from a non-blocking write, and then call
    // the rest after an interrupt

    // Instead, we are going to block here until we get TX_DS (transmission completed and ack'd)
    // or MAX_RT (maximum retries, transmission failed).  Also, we'll timeout in case the radio
    // is flaky and we get neither.

    // IN the end, the send should be blocking.  It comes back in 60ms worst case, or much faster
    // if I tighted up the retry logic.  (Default settings will be 1500us.
    // Monitor the send
    quint8 observe_tx;
    quint8 status;
    uint32_t sent_at = millis();
    const unsigned long timeout = 500; //ms to wait for timeout
    do
    {
        status = readRegister(OBSERVE_TX,&observe_tx,1);

        //if (debug)
           // qDebug() << QString ("Observe_Tx=%1").arg(observe_tx);
            //printf("%02X", observe_tx);
    }
    while( ! ( status & ( _BV(TX_DS) | _BV(MAX_RT) ) ) && ( millis() - sent_at < timeout ) );


    // The part above is what you could recreate with your own interrupt handler,
    // and then call this when you got an interrupt
    // -----------
    // Call this when you get an interrupt
    // The status tells us three things
    // * The send was successful (TX_DS)
    // * The send failed, too many retries (MAX_RT)
    // * There is an ack packet waiting (RX_DR)
    bool tx_ok, tx_fail;

    whatHappened(tx_ok,tx_fail,ack_payload_available);

    qDebug() << QString("tx_ok=%1,tx_fail=%2,ack_payload_avialable=%3").arg(tx_ok).arg(tx_fail).arg(ack_payload_available);

    result = tx_ok;
    //if (debug)
        qDebug() << (result?"...OK.":"...Failed");

    // Handle the ack packet
    if ( ack_payload_available )
    {
        ack_payload_length = getDynamicPayloadSize();
       // if (debug)
        //    qDebug() << QString("[AckPacket]/%1").arg(ack_payload_length);
    }

    // Yay, we are done.

    // Power down
//    powerDown();

    // Flush buffers (Is this a relic of past experimentation, and not needed anymore??)
    flushTx();

    return result;
}

void QRF24::startWrite( const void* buf, quint8 len )
{
    // Transmitter power-up
    writeRegister(CONFIG, ( readRegister(CONFIG) | _BV(PWR_UP) ) & ~_BV(PRIM_RX) );
    delayMicroseconds(150);

    // Send the payload
    writePayload( buf, len );

    // Allons!
    bcm2835_gpio_write(ce_pin, HIGH);
    delayMicroseconds(15);
    bcm2835_gpio_write(ce_pin, LOW);
}
quint8 QRF24::getDynamicPayloadSize()
{
    quint8 result = 0;

    bcm2835_spi_transfer( R_RX_PL_WID );
    result = bcm2835_spi_transfer(0xff);

    return result;
}
bool QRF24::available()
{
    return available(NULL);
}
bool QRF24::available(quint8* pipe_num)
{
    quint8 status = getStatus();

    // Too noisy, enable if you really want lots o data!!
    // if (debug) print_status(status);

    bool result = ( status & _BV(RX_DR) );

    if (result)
    {
        // If the caller wants the pipe number, include that
        if ( pipe_num )
            *pipe_num = ( status >> RX_P_NO ) & 0b111;

        // Clear the status bit

        // ??? Should this REALLY be cleared now?  Or wait until we
        // actually READ the payload?

        writeRegister(STATUS,_BV(RX_DR) );

        // Handle ack payload receipt
        if ( status & _BV(TX_DS) )
        {
            writeRegister(STATUS,_BV(TX_DS));
        }
    }
    //if(result) qDebug() << result;
    //if (result) qDebug() << QString ("available %1").arg(result);
    return result;
}
bool QRF24::read( void* buf, quint8 len )
{
    // Fetch the payload
    readPayload( buf, len );
    //qDebug() << QString ("ReadPayload:buf=%1,len=%2").arg(*buf).arg(len);

    // was this the last of the data available?
    return readRegister(FIFO_STATUS) & _BV(RX_EMPTY);
}

void QRF24::whatHappened(bool& tx_ok,bool& tx_fail,bool& rx_ready)
{
    // Read the status & reset the status in one easy call
    // Or is that such a good idea?
    quint8 status = writeRegister(STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

    // Report to the user what happened
    tx_ok = status & _BV(TX_DS);
    tx_fail = status & _BV(MAX_RT);
    rx_ready = status & _BV(RX_DR);
}

void QRF24::openWritingPipe(quint64 value)
{
    // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
    // expects it LSB first too, so we're good.

    writeRegister(RX_ADDR_P0, reinterpret_cast<quint8*>(&value), 5);
    writeRegister(TX_ADDR, reinterpret_cast<quint8*>(&value), 5);

    const quint8 max_payload_size = 32;
    writeRegister(RX_PW_P0,qMin(payload_size,max_payload_size));
}
static const quint8 child_pipe[] =
{
    RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const quint8 child_payload_size[] =
{
    RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};
static const quint8 child_pipe_enable[] =
{
    ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};
void QRF24::openReadingPipe(quint8 child, quint64 address)
{
    // If this is pipe 0, cache the address.  This is needed because
    // openWritingPipe() will overwrite the pipe 0 address, so
    // startListening() will have to restore it.
    if (child == 0) writeRegister(RX_ADDR_P0, reinterpret_cast<const quint8*>(&mypipe0), 5);
/*
    if (child == 0)
        pipe0_reading_address = address;

    if (child <= 6)
    {
        // For pipes 2-5, only write the LSB
        if ( child < 2 )
            writeRegister(child_pipe[child], reinterpret_cast<const quint8*>(&address), 5);
        else
            writeRegister(child_pipe[child], reinterpret_cast<const quint8*>(&address), 1);

        writeRegister(child_payload_size[child],payload_size);

        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.
        writeRegister(EN_RXADDR,readRegister(EN_RXADDR) | _BV(child_pipe_enable[child]));
    }*/
}
void QRF24::toggleFeatures()
{
    bcm2835_spi_transfer( ACTIVATE );
    bcm2835_spi_transfer( 0x73 );
}
void QRF24::enableDynamicPayloads()
{
    // Enable dynamic payload throughout the system
    writeRegister(FEATURE,readRegister(FEATURE) | _BV(EN_DPL) );

    // If it didn't work, the features are not enabled
    if ( ! readRegister(FEATURE) )
    {
        // So enable them and try again
        toggleFeatures();
        writeRegister(FEATURE,readRegister(FEATURE) | _BV(EN_DPL) );
    }

    if (debug)
        printf("FEATURE=%i\r\n",readRegister(FEATURE));

    // Enable dynamic payload on all pipes
    //
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    writeRegister(DYNPD,readRegister(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));

    dynamic_payloads_enabled = true;
}



void QRF24::enableAckPayload()
{
    //
    // enable ack payload and dynamic payload features
    //

    writeRegister(FEATURE,readRegister(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );

    // If it didn't work, the features are not enabled
    if ( ! readRegister(FEATURE) )
    {
        // So enable them and try again
        toggleFeatures();
        writeRegister(FEATURE,readRegister(FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL) );
    }

    //if (debug)
        //qDebug() << QString ("FEATURE=%1").arg(readRegister(FEATURE));

    //
    // Enable dynamic payload on pipes 0 & 1
    //

    writeRegister(DYNPD,readRegister(DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
}
void QRF24::writeAckPayload(quint8 pipe, const void* buf, quint8 len)
{
    const quint8* current = reinterpret_cast<const quint8*>(buf);

    bcm2835_spi_transfer( W_ACK_PAYLOAD | ( pipe & 0b111 ) );
    const quint8 max_payload_size = 32;
    quint8 data_len = qMin(len,max_payload_size);
    while ( data_len-- )
        bcm2835_spi_transfer(*current++);
    //qDebug() << ("writeAckPayload");
}



bool QRF24::isAckPayloadAvailable()
{
    bool result = ack_payload_available;
    ack_payload_available = false;
    return result;
    //qDebug() << QString ("isAckPayloadAvialable=%1").arg(result);
}



bool QRF24::isPVariant()
{
    return p_variant ;
}



void QRF24::setAutoAck(bool enable)
{
    if ( enable )
        writeRegister(EN_AA, 0b111111);
    else
        writeRegister(EN_AA, 0);
}



void QRF24::setAutoAck( quint8 pipe, bool enable )
{
    if ( pipe <= 6 )
    {
        quint8 en_aa = readRegister( EN_AA ) ;
        if( enable)
        {
            en_aa |= _BV(pipe) ;
        }
        else
        {
            en_aa &= ~_BV(pipe) ;
        }
        writeRegister( EN_AA, en_aa ) ;
    }
}
bool QRF24::testCarrier()
{
    return ( readRegister(CD) & 1 );
}
bool QRF24::testRPD()
{
    return ( readRegister(RPD) & 1 ) ;
}
void QRF24::setPALevel(rf24_pa_dbm_e level)
{
    quint8 setup = readRegister(RF_SETUP) ;
    setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

    // switch uses RAM (evil!)
    if ( level == RF24_PA_MAX )
    {
        setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
    }
    else if ( level == RF24_PA_HIGH )
    {
        setup |= _BV(RF_PWR_HIGH) ;
    }
    else if ( level == RF24_PA_LOW )
    {
        setup |= _BV(RF_PWR_LOW);
    }
    else if ( level == RF24_PA_MIN )
    {
        // nothing
    }
    else if ( level == RF24_PA_ERROR )
    {
        // On error, go to maximum PA
        setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
    }

    writeRegister( RF_SETUP, setup ) ;
}
rf24_pa_dbm_e QRF24::getPALevel()
{
    rf24_pa_dbm_e result = RF24_PA_ERROR ;
    quint8 power = readRegister(RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

    // switch uses RAM (evil!)
    if ( power == (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) )
    {
        result = RF24_PA_MAX ;
    }
    else if ( power == _BV(RF_PWR_HIGH) )
    {
        result = RF24_PA_HIGH ;
    }
    else if ( power == _BV(RF_PWR_LOW) )
    {
        result = RF24_PA_LOW ;
    }
    else
    {
        result = RF24_PA_MIN ;
    }

    return result ;
}
bool QRF24::setDataRate(rf24_datarate_e speed)
{
    bool result = false;
    quint8 setup = readRegister(RF_SETUP) ;

    // HIGH and LOW '00' is 1Mbs - our default
    wide_band = false ;
    setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
    if( speed == RF24_250KBPS )
    {
        // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
        // Making it '10'.
        wide_band = false ;
        setup |= _BV( RF_DR_LOW ) ;
    }
    else
    {
        // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
        // Making it '01'
        if ( speed == RF24_2MBPS )
        {
            wide_band = true ;
            setup |= _BV(RF_DR_HIGH);
        }
        else
        {
            // 1Mbs
            wide_band = false ;
        }
    }
    writeRegister(RF_SETUP,setup);

    // Verify our result
    if ( readRegister(RF_SETUP) == setup )
    {
        result = true;
    }
    else
    {
        wide_band = false;
    }

    return result;
}
rf24_datarate_e QRF24::getDataRate( void )
{
    rf24_datarate_e result ;
    quint8 dr = readRegister(RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    // switch uses RAM (evil!)
    // Order matters in our case below
    if ( dr == _BV(RF_DR_LOW) )
    {
        // '10' = 250KBPS
        result = RF24_250KBPS ;
    }
    else if ( dr == _BV(RF_DR_HIGH) )
    {
        // '01' = 2MBPS
        result = RF24_2MBPS ;
    }
    else
    {
        // '00' = 1MBPS
        result = RF24_1MBPS ;
    }
    return result ;
}



void QRF24::setCRCLength(rf24_crclength_e length)
{
    quint8 config = readRegister(CONFIG) & ~( _BV(CRCO) | _BV(EN_CRC)) ;

    // switch uses RAM (evil!)
    if ( length == RF24_CRC_DISABLED )
    {
        // Do nothing, we turned it off above.
    }
    else if ( length == RF24_CRC_8 )
    {
        config |= _BV(EN_CRC);
    }
    else
    {
        config |= _BV(EN_CRC);
        config |= _BV( CRCO );
    }
    writeRegister( CONFIG, config ) ;
}



rf24_crclength_e QRF24::getCRCLength()
{
    rf24_crclength_e result = RF24_CRC_DISABLED;
    quint8 config = readRegister(CONFIG) & ( _BV(CRCO) | _BV(EN_CRC)) ;

    if ( config & _BV(EN_CRC ) )
    {
        if ( config & _BV(CRCO) )
            result = RF24_CRC_16;
        else
            result = RF24_CRC_8;
    }

    return result;
}



void QRF24::disableCRC( void )
{
    quint8 disable = readRegister(CONFIG) & ~_BV(EN_CRC) ;
    writeRegister( CONFIG, disable ) ;
}


void QRF24::setRetries(quint8 delay, quint8 count)
{
    writeRegister(SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}


