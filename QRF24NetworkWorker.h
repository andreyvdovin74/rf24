/*
 Copyright (C) 2014 Benoit Dumas <bntdumas@gmail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#ifndef QRF24NETWORKWORKER_H
#define QRF24NETWORKWORKER_H

#include "QRF24Network.h"

#include <QObject>
#include <QVector>
#include <QPair>


/**
 * @brief Handle the communication loop for the radio module.
 */

class QRF24;

class QRF24NetworkWorker : public QObject
{
    Q_OBJECT


public:
    /**
     * @brief QRF24NetworkWorker
     */
    explicit QRF24NetworkWorker();

    /**
     * @brief Set the time spent listening for new data.
     * @param ms: the amount of time in milliseconds
     * @note: default is 500ms
     */
    void setListeningTime(const unsigned int ms);

Q_SIGNALS:
    /**
     * @brief Emitted when the thread exits the main loop.
     * @param message: a human redable message giving the reason
     * @param error: true if the thread died because of an error
     */
    void finished(const QString &message, const bool error);

    /**
     * @brief emitted when new data is available.
     * @param data: the received data
     * @param pipe: the pipe on which the data is aavilable
     */
    void newData(const QString &data, quint64 emittingNode);

    /**
     * @brief Emitted if an error occured
     * @param message: a text describing the error
     */
    void error(const QString &message);

public Q_SLOTS:
    /**
     * @brief Thread main loop.
     * - Listen for a determined amount of time (set with \b setListeningTime), if data is available on either of the pipes,
     *   retreive the data and it's corresponding pipe and emit \b newData(const QString &data, quint64 pipe)
     * - After the listening, check for new data to write on the pipes.
     *
     * @param radioModule: a configured QRF24 radio module
     */
    void mainRadioLoop(QRF24Network *networkModule);

    /**
     * @brief Write data to the passed pipe (using Queue)
     * @param data: the data to write
     * @param pipe: the pipe to write into
     */
    void requestWrite(const QString &data, const quint64 pipe);

    /**
     * @brief Ask the thread to finish execution.
     */
    void stop();

private:
    /**
     * @brief A queue containing the data to write into the radio pipes
     */
    QVector< QPair<quint64, QString> > m_writeQueue;

    /**
     * @brief used to exit the main loop on user request.
     */
    bool m_stopRequested;

    /**
     * @brief Time spent listening for new data available for each loop iteration
     */
    int m_listeningTime;

    /**
     * @brief Address of this node on the RF24 network.
     */
    quint16 m_nodeAddress;

    /**
     * @brief Listen for new data on the pipes and emit \b if available.
     * @param radioModule: the networked radio module instance
     */
    void listenForNewData(QRF24Network *networkNode);

    /**
     * @brief write all data from the \b m_writeQueue queue.
     * @param radioModule: the radio module instance
     */
    void write(QRF24 *radioModule);

    /**
     * @brief determines if the thread needs to suspend read/write operations
     */
    bool m_suspendThread;


};

#endif // QRF24NetworkWorker_H
