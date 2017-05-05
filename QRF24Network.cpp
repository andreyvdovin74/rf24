/*
 Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.

 05/2014: Benoit Dumas <bntdumas@gmail.com>
    Modified to use the Qt framework and added a loop running on
    a separate thread to write/read from the pipes
 */

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <iostream>  
#include <algorithm>

#include <QDebug>

#include "QRF24Network_config.h"
#include "QRF24.h"
#include "QRF24Network.h"
#include "QRF24NetworkWorker.h"

quint16 QRF24NetworkHeader::next_id = 1;

QRF24Network::QRF24Network( QRF24& _radio, QObject *parent):
    QObject(parent),
    radio(_radio),
    next_frame(frame_queue)
{
}

QRF24Network::~QRF24Network()
{
    terminateRadioModule();
}

void QRF24Network::begin(quint8 _channel, quint16 _node_address )
{
    if (! isValidAddress(_node_address) )
        return;

    node_address = _node_address;

    // Set up the radio the way we want it to look
    radio.setAutoAck(true);
    radio.setChannel(_channel);
    radio.setDataRate(RF24_1MBPS);
    radio.setCRCLength(RF24_CRC_16);
    radio.openReadingPipe(0,mypipe0);
    radio.openReadingPipe(1,mypipe1);
    radio.openWritingPipe(mypipe0);
    // Setup our address helper cache
    setupAddress();

    // Open up all listening pipes
//    int i = 6;
//    while (i--) {
//    radio.openReadingPipe(i,pipeAddress(_node_address,i));
   // radio.openWritingPipe(pipeAddress(_node_address,i));
//    }
    //radio.openReadingPipe(0,mypipe);

    //prepare the main loop for writing/reading frames
    m_worker = new QRF24NetworkWorker();
    m_worker->moveToThread(&m_moduleThread);
    connect(&m_moduleThread, SIGNAL(finished()), m_worker, SLOT(deleteLater()));
//    connect(&m_moduleThread::finished(),m_worker->deleteLater());
//    connect(m_worker, SIGNAL(newData(QString,quint64)), this, SLOT(requestWrite(QString,quint64)));
    connect(this, SIGNAL(startThread(QRF24Network*)), m_worker, SLOT(mainRadioLoop(QRF24Network*)));
    m_moduleThread.start();

    Q_EMIT startThread(this);

    radio.startListening();
}

void QRF24Network::update()
{
    // if there is data ready
    quint8 pipe_num;
    while ( radio.available(&pipe_num) )
    {
        // Dump the payloads until we've gotten everything
        bool done = false;
        while (!done)
        {
            // Fetch the payload, and see if this was the last one.
            done = radio.read( frame_buffer, sizeof(frame_buffer) );

//            // Read the beginning of the frame as the header
//            const QRF24NetworkHeader& header = * reinterpret_cast<QRF24NetworkHeader*>(frame_buffer);

//            // Throw it away if it's not a valid address
//            if ( !isValidAddress(header.to_node) )
//                continue;

//            // Is this for us?
//            if ( header.to_node == node_address )
//                // Add it to the buffer of frames for us
//                enqueue();
//            else
//                // Relay it
//                write(header.to_node);
        }
    }
}

bool QRF24Network::enqueue()
{
    bool result = false;

    // Copy the current frame into the frame queue
    if ( next_frame < frame_queue + sizeof(frame_queue) )
    {
        memcpy(next_frame,frame_buffer, frame_size );
        next_frame += frame_size;
        result = true;
    }

    return result;
}

bool QRF24Network::available()
{
    // Are there frames on the queue for us?
    return (next_frame > frame_queue);
}

quint16 QRF24Network::parent() const
{
    if ( node_address == 0 )
        return -1;
    else
        return parent_node;
}

QRF24 *QRF24Network::radioModule()
{
    return &radio;
}

void QRF24Network::peek(QRF24NetworkHeader& header)
{
    if ( available() )
    {
        // Copy the next available frame from the queue into the provided buffer
        memcpy(&header,next_frame-frame_size,sizeof(QRF24NetworkHeader));
    }
}

size_t QRF24Network::read(QRF24NetworkHeader& header, void* message, size_t maxlen)
{
    size_t bufsize = 0;

//    if ( available() )
//    {
        // Move the pointer back one in the queue
        next_frame -= frame_size;
        quint8* frame = next_frame;

        // How much buffer size should we actually copy?
        bufsize = qMin(maxlen,frame_size-sizeof(QRF24NetworkHeader));

        // Copy the next available frame from the queue into the provided buffer
        memcpy(&header,frame,sizeof(QRF24NetworkHeader));
        memcpy(message,frame+sizeof(QRF24NetworkHeader),bufsize);
       // qDebug() << message;

//    }

    return bufsize;
}

bool QRF24Network::write(QRF24NetworkHeader& header,const void* message, size_t len)
{
    // Fill out the header
    header.from_node = node_address;

    // Build the full frame to send
    memcpy(frame_buffer,&header,sizeof(QRF24NetworkHeader));
    if (len)
        memcpy(frame_buffer + sizeof(QRF24NetworkHeader),message,qMin(frame_size-sizeof(QRF24NetworkHeader),len));

    // If the user is trying to send it to himself
    if ( header.to_node == node_address )
        // Just queue it in the received queue
        return enqueue();
    else
        // Otherwise send it out over the air
        return write(header.to_node);
}


bool QRF24Network::write(quint16 to_node)
{
    bool ok = false;

    // Throw it away if it's not a valid address
    if ( !isValidAddress(to_node) )
        return false;

    // First, stop listening so we can talk.
    radio.stopListening();

    // Where do we send this?  By default, to our parent
    quint16 send_node = parent_node;
    // On which pipe
    quint8 send_pipe = parent_pipe;

    // If the node is a direct child,
    if ( isDirectChild(to_node) )
    {
        // Send directly
        send_node = to_node;

        // To its listening pipe
        send_pipe = 0;
    }
    // If the node is a child of a child
    // talk on our child's listening pipe,
    // and let the direct child relay it.
    else if ( isDescendant(to_node) )
    {
        send_node = directChildRouteTo(to_node);
        send_pipe = 0;
    }

    // First, stop listening so we can talk
    radio.stopListening();

    // Put the frame on the pipe
    ok = writeToPipe( send_node, send_pipe );

    // NOT NEEDED anymore.  Now all reading pipes are open to start.
#if 0
    // If we are talking on our talking pipe, it's possible that no one is listening.
    // If this fails, try sending it on our parent's listening pipe.  That will wake
    // it up, and next time it will listen to us.

    if ( !ok && send_node == parent_node )
        ok = write_to_pipe( parent_node, 0 );
#endif

    // Now, continue listening
    radio.startListening();

    return ok;
}

bool QRF24Network::writeToPipe( quint16 node, quint8 pipe )
{
    bool ok = false;

    quint64 out_pipe = pipeAddress( node, pipe );

    // Open the correct pipe for writing.
    radio.openWritingPipe(out_pipe);

    // Retry a few times
    short attempts = 5;
    do
    {
        ok = radio.write( frame_buffer, frame_size );
    }
    while ( !ok && --attempts );
    return ok;
}

const char* QRF24NetworkHeader::toString() const
{
    static char buffer[45];
    //snprintf_P(buffer,sizeof(buffer),PSTR("id %04x from 0%o to 0%o type %c"),id,from_node,to_node,type);
    return buffer;
}

bool QRF24Network::isDirectChild( quint16 node )
{
    bool result = false;

    // A direct child of ours has the same low numbers as us, and only
    // one higher number.
    //
    // e.g. node 0234 is a direct child of 034, and node 01234 is a
    // descendant but not a direct child

    // First, is it even a descendant?
    if ( isDescendant(node) )
    {
        // Does it only have ONE more level than us?
        quint16 child_node_mask = ( ~ node_mask ) << 3;
        result = ( node & child_node_mask ) == 0 ;
    }

    return result;
}

bool QRF24Network::isDescendant( quint16 node )
{
    return ( node & node_mask ) == node_address;
}

void QRF24Network::setupAddress()
{
    // First, establish the node_mask
    quint16 node_mask_check = 0xFFFF;
    while ( node_address & node_mask_check )
        node_mask_check <<= 3;

    node_mask = ~ node_mask_check;

    // parent mask is the next level down
    quint16 parent_mask = node_mask >> 3;

    // parent node is the part IN the mask
    parent_node = node_address & parent_mask;

    // parent pipe is the part OUT of the mask
    quint16 i = node_address;
    quint16 m = parent_mask;
    while (m)
    {
        i >>= 3;
        m >>= 3;
    }
    parent_pipe = i;
}

quint16 QRF24Network::directChildRouteTo( quint16 node )
{
    // Presumes that this is in fact a child!!

    quint16 child_mask = ( node_mask << 3 ) | 0B111;
    return node & child_mask ;
}

quint8 QRF24Network::pipeToDescendant( quint16 node )
{
    quint16 i = node;
    quint16 m = node_mask;

    while (m)
    {
        i >>= 3;
        m >>= 3;
    }

    return i & 0B111;
}

void QRF24Network::requestWrite(const QString &data, const quint64 pipe)
{
    m_worker->requestWrite(data, pipe);
}

void QRF24Network::terminateRadioModule()
{
    m_worker->stop();
    m_moduleThread.exit();
    m_moduleThread.wait();
//#ifdef TESTING
//    m_terminated = true;
//#endif
}

bool QRF24Network::isValidAddress( quint16 node )
{
    bool result = true;

    while(node)
    {
        quint8 digit = node & 0B111;
        if (digit < 1 || digit > 5)
        {
            result = false;
           // printf_P(PSTR("*** WARNING *** Invalid address 0%o\n\r"),node);
            break;
        }
        node >>= 3;
    }
    qDebug() << QString ("nodeAddr %1").arg(result);
    return result;
}

quint64 QRF24Network::pipeAddress( quint16 node, quint8 pipe )
{
    static quint8 pipe_segment[] = { 0x3c, 0x5a, 0x69, 0x96, 0xa5, 0xc3 };

    quint64 result;
    quint8* out = reinterpret_cast<quint8*>(&result);

    out[0] = pipe_segment[pipe];

    quint8 w;
    short i = 4;
    short shift = 12;
    while(i--)
    {
        w = ( node >> shift ) & 0xF ;
        w |= ~w << 4;
        out[i+1] = w;

        shift -= 4;
    }
    qDebug() << result;
    return result;
    //qDebug() << result;
}
