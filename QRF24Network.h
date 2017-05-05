/*
 Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.

 05/2014: Benoit Dumas <bntdumas@gmail.com>
                Modified to use the Qt framework and added a loop running on
                a separate thread to write/read from the pipes
 */

#ifndef __RF24NETWORK_H__
#define __RF24NETWORK_H__

/**
 * @file QRF24Network.h
 * Class declaration for RF24Network
 */

//#define TESTING

#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <sys/time.h>
#include <stddef.h>

#include <QObject>
#include <QThread>

#ifdef TESTING
#include <QTimer>
#endif

class QRF24;
class QRF24NetworkWorker;

/**
 * Header which is sent with each message
 * The frame put over the air consists of this header and a message
 */
struct QRF24NetworkHeader
{
    quint16 from_node; /**< Logical address where the message was generated */
    quint16 to_node; /**< Logical address where the message is going */
    quint16 id; /**< Sequential message ID, incremented every message */
    unsigned char type; /**< Type of the packet.  0-127 are user-defined types, 128-255 are reserved for system */
    unsigned char reserved; /**< Reserved for future use */

    static quint16 next_id; /**< The message ID of the next message to be sent */

    /**
   * Default constructor
   *
   * Simply constructs a blank header
   */
    QRF24NetworkHeader() {}

    /**
   * Send constructor
   *
   * Use this constructor to create a header and then send a message
   *
   * @code
   *  RF24NetworkHeader header(recipient_address,'t');
   *  network.write(header,&message,sizeof(message));
   * @endcode
   *
   * @param _to The logical node address where the message is going
   * @param _type The type of message which follows.  Only 0-127 are allowed for
   * user messages.
   */
    QRF24NetworkHeader(quint16 _to, unsigned char _type = 0): to_node(_to), id(next_id++), type(_type&0x7f) {}

    /**
   * Create debugging string
   *
   * Useful for debugging.  Dumps all members into a single string, using
   * internal static memory.  This memory will get overridden next time
   * you call the method.
   *
   * @return String representation of this object
   */
    const char* toString() const;
};

/**
 * Network Layer for RF24 Radios
 *
 * This class implements an OSI Network Layer using nRF24L01(+) radios driven
 * by RF24 library.
 */

class QRF24Network: public QObject
{
    Q_OBJECT

public:

    /**
   * Construct the network
   * @param _radio The underlying radio driver instance
   */
    explicit QRF24Network( QRF24& _radio, QObject *parent = 0);
    ~QRF24Network();

    /**
   * Bring up the network
   *
   * @warning Be sure to 'begin' the radio first.
   *
   * @param _channel The RF channel to operate on
   * @param _node_address The logical address of this node
   */
    void begin(quint8 _channel, quint16 _node_address );
    //void begin_RX(quint8 _channel, quint16 node_address);
    /**
   * This node's parent address
   *
   * @return This node's parent address, or -1 if this is the base
   */
    quint16 parent() const;

    /**
   * @brief returns the underlying radio module
   */
    QRF24 *radioModule();

public Q_SLOTS:
    /**
     * @brief Write data to the passed pipe (using Queue)
     * @param data: the data to write
     * @param pipe: the pipe to write into
     */
    void requestWrite(const QString &data, const quint64 pipe);

    /**
     * @brief Stop the radio module loop and destroy the thread.
     * @note delete the RF24* passed to the main loop.
     */
    void terminateRadioModule();

Q_SIGNALS:
    /**
     * @brief Emitted if an error occured
     * @param message: a text describing the error
     */
    void error(const QString &message);

    /**
     * @brief emitted when new data is available.
     * @param data: the received data
     * @param pipe: the pipe on which the data is aavilable
     */
    void newData(const QString &data, quint64 pipe);

    /**
     * @brief Starts the main loop of the radio module thread
     */
    void startThread(QRF24Network *radioModule);

protected:
    /**
     * Main layer loop
     *
     * This function must be called regularly to keep the layer going.  This is where all
     * the action happens!
     */
    void update();

    /**
     * Test whether there is a message available for this node
     *
     * @return Whether there is a message available for this node
     */
    bool available();

    /**
     * Read the next available header
     *
     * Reads the next available header without advancing to the next
     * incoming message.  Useful for doing a switch on the message type
     *
     * If there is no message available, the header is not touched
     *
     * @param[out] header The header (envelope) of the next message
     */
    void peek(QRF24NetworkHeader& header);

    /**
     * Read a message
     *
     * @param[out] header The header (envelope) of this message
     * @param[out] message Pointer to memory where the message should be placed
     * @param maxlen The largest message size which can be held in @p message
     * @return The total number of bytes copied into @p message
     */
    size_t read(QRF24NetworkHeader& header, void* message, size_t maxlen);

    /**
     * Send a message
     *
     * @param[in,out] header The header (envelope) of this message.  The critical
     * thing to fill in is the @p to_node field so we know where to send the
     * message.  It is then updated with the details of the actual header sent.
     * @param message Pointer to memory where the message is located
     * @param len The size of the message
     * @return Whether the message was successfully received
     */
    bool write(QRF24NetworkHeader& header,const void* message, size_t len);



//    void openPipes();
    quint16 findNode( quint16 current_node, quint16 target_node );
    bool write(quint16);
    bool writeToPipe( quint16 node, quint8 pipe );
    bool enqueue();

    bool isDirectChild( quint16 node );
    bool isDescendant( quint16 node );
    quint16 directChildRouteTo( quint16 node );
    quint8 pipeToDescendant( quint16 node );
    void setupAddress();

    bool isValidAddress(quint16 node);
    quint64 pipeAddress(quint16 node, quint8 pipe);

private:

    QRF24& radio; /**< Underlying radio driver, provides link/physical layers */
    quint16 node_address; /**< Logical node address of this unit, 1 .. UINT_MAX */
    const static int frame_size = 32; /**< How large is each frame over the air */
    quint8 frame_buffer[frame_size]; /**< Space to put the frame that will be sent/received over the air */
    quint8 frame_queue[5*frame_size]; /**< Space for a small set of frames that need to be delivered to the app layer */
    quint8* next_frame; /**< Pointer into the @p frame_queue where we should place the next received frame */

    quint16 parent_node; /**< Our parent's node address */
    quint8 parent_pipe; /**< The pipe our parent uses to listen to us */
    quint16 node_mask; /**< The bits which contain signfificant node address information */
    //const uint64_t mypipe = 0xF0F1F2F3F4LL;
    const uint64_t mypipe0 = 0xE7E7E7E7E7UL;
    const uint64_t mypipe1 = 0xE7E7E7E7E8UL;
    /**
   * @brief The radio module loop members
   */
    QThread m_moduleThread;
    QRF24NetworkWorker *m_worker;

    // hide the RF24 API  from the user but leave it accessible to the worker. Enforces uses of Qt methods.
    friend class QRF24NetworkWorker;


};

#endif // __RF24NETWORK_H__
