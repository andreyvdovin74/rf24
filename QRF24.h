#ifndef __RF24_H__
#define __RF24_H__

#include "QRF24Network_config.h"
#include "./bcm2835.h"

#include <QObject>

typedef enum {
    RF24_PA_MIN = 0,
    RF24_PA_LOW,
    RF24_PA_HIGH,
    RF24_PA_MAX,
    RF24_PA_ERROR
} rf24_pa_dbm_e ;

/**
 * Data rate.  How fast data moves through the air.
 *
 * For use with setDataRate()
 */
typedef enum {
    RF24_1MBPS = 0,
    RF24_2MBPS,
    RF24_250KBPS
} rf24_datarate_e;

typedef enum {
    RF24_CRC_DISABLED = 0,
    RF24_CRC_8,
    RF24_CRC_16
} rf24_crclength_e;

class QRF24: public QObject
{
    Q_OBJECT

public:
    explicit QRF24(quint8 _cepin, quint8 _cspin, QObject *parent = 0);
    explicit QRF24(quint8 _cepin, quint8 _cspin, uint32_t spispeed, QObject *parent = 0);

    bool begin();
    void startListening();
    void stopListening();
    bool write( const void* buf, quint8 len );
    bool available();
    bool read( void* buf, quint8 len );
    void openWritingPipe(quint64 address);
    void openReadingPipe(quint8 number, quint64 address);
    void setRetries(quint8 delay, quint8 count);
    void setChannel(quint8 channel);
    void setPayloadSize(quint8 size);
    quint8 getPayloadSize();
    quint8 getDynamicPayloadSize();
    void enableAckPayload();
    void enableDynamicPayloads();
    bool isPVariant() ;
    void setAutoAck(bool enable);
    void setAutoAck( quint8 pipe, bool enable ) ;
    void setPALevel( rf24_pa_dbm_e level ) ;
    rf24_pa_dbm_e getPALevel( void ) ;
    bool setDataRate(rf24_datarate_e speed);
    rf24_datarate_e getDataRate( void ) ;
    void setCRCLength(rf24_crclength_e length);
    rf24_crclength_e getCRCLength();
    void disableCRC( void ) ;
    void getDetails(bool);
    void powerDown();
    void powerUp() ;
    bool available(quint8* pipe_num);
    void startWrite( const void* buf, quint8 len );
    void writeAckPayload(quint8 pipe, const void* buf, quint8 len);
    bool isAckPayloadAvailable();
    void whatHappened(bool& tx_ok,bool& tx_fail,bool& rx_ready);
    bool testRPD();
    bool testCarrier();
    quint8 getStatus();
private:
    quint8 ce_pin; /**< "Chip Enable" pin, activates the RX or TX role */
    quint8 csn_pin; /**< SPI Chip select */
    quint16 spi_speed; /**< SPI Bus Speed */
    bool wide_band; /* 2Mbs data rate in use? */
    bool p_variant; /* False for RF24L01 and true for RF24L01P */
    quint8 payload_size; /**< Fixed size of payloads */
    bool ack_payload_available; /**< Whether there is an ack payload waiting */
    bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
    quint8 ack_payload_length; /**< Dynamic size of pending ack payload. */
    quint64 pipe0_reading_address; /**< Last address set on pipe 0 for reading. */
    quint8 debug ; /* Debug flag */
    quint8 spi_rxbuff[32] ; //SPI receive buffer (payload max 32 bytes)
    quint8 spi_txbuff[32+1] ; //SPI transmit buffer (payload max 32 bytes + 1 byte for the command)

protected:
    const quint64 mypipe0 = 0xE7E7E7E7E7UL;
    quint8 readRegister(quint8 reg, quint8* buf, quint8 len);
    quint8 readRegister(quint8 reg);
    quint8 writeRegister(quint8 reg, const quint8* buf, quint8 len);
    quint8 writeRegister(quint8 reg, quint8 value);
    quint8 writePayload(const void* buf, quint8 len);
    quint8 readPayload(void* buf, quint8 len);
    quint8 flushRx();
    quint8 flushTx();
protected:
    QString status(quint8 status) const;
    QString printObserveTx(quint8 value) const;
    QString byteRegister(const char* name, quint8 reg, quint8 qty = 1);
    QString addressRegister(const char* name, quint8 reg, quint8 qty = 1);
    void toggleFeatures();
};
#endif // __RF24_H__

