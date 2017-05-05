/*
 Copyright (C) 2014 Benoit Dumas <bntdumas@gmail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include "QRF24NetworkWorker.h"
#include "QRF24.h"

#include <QTimer>
#include <QByteArray>
#include <QDebug>
#include <QtGlobal>

QRF24NetworkWorker::QRF24NetworkWorker() :
    QObject(),
    m_stopRequested(false),
    m_listeningTime(500)
{
}

void QRF24NetworkWorker::setListeningTime(const unsigned int ms)
{
    m_listeningTime = ms;
}

void QRF24NetworkWorker::mainRadioLoop(QRF24Network *networkModule)
{
    QRF24 *radioModule = networkModule->radioModule();

    while (!m_stopRequested) {

        networkModule->update();

        // Listen for new data
        for (int i = 0; i < 100; ++i) {
            listenForNewData(networkModule);
        }

        // Write data

        while (m_writeQueue.count())    {
            write(radioModule);
//         qDebug() << QString("Queue %1").arg(m_writeQueue.count());
    }
    }
    Q_EMIT finished(tr("Radio module thread finished."), false);
}

void QRF24NetworkWorker::requestWrite(const QString &data, const quint64 pipe)
{
    QPair<quint64, QString> payload;
    payload.first = pipe;
    payload.second = data;
    //qDebug() << QString ("pipe=%1 data=%2").arg(pipe,2,16).arg(data);
    m_writeQueue.append(payload);
}


void QRF24NetworkWorker::stop()
{
    qDebug() << Q_FUNC_INFO;
    m_stopRequested = true;
}

void QRF24NetworkWorker::listenForNewData(QRF24Network *networkNode)
{
    if (networkNode->available()) {
        qDebug() << "Data available!";
        QRF24NetworkHeader header;
        char buff[1024];
        int  size = networkNode->read(header, buff, sizeof(buff));
        //qDebug() << QString ("Size=%1").arg(size);
        if (!size) {
            Q_EMIT error(tr("Reading failed."));
        }
             Q_EMIT newData(QString(buff), header.from_node);
        //qDebug() << size;
    }
//    qDebug() << QByteArray(*(QRF24Network.frame_buffer),sizeof(QRF24Network.frame_buffer)).toHex();
    delayMicroseconds(m_listeningTime * 1000 / 100);
}

void QRF24NetworkWorker::write(QRF24 *radioModule)
{
    for (int i = 0; i < m_writeQueue.count(); ++i) {
        QPair<quint64, QString> data = m_writeQueue.at(i);
//        qDebug() << QString("pipe=0x%1,data=%2").arg(data.first,8,16,QChar('0')).arg(data.second);
        radioModule->openWritingPipe(data.first);
        if (!radioModule->write(data.second.toStdString().c_str(), data.second.count())) {
            Q_EMIT error(tr("Writing to pipe %1 failed.").arg(QString::number(data.first, 16)));
        }
        m_writeQueue.remove(i);
    }
}
