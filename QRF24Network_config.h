
/*
 Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.

 05/2014: Benoit Dumas <bntdumas@gmail.com>
                Modified to use the Qt framework.

 */

#ifndef __RF24_CONFIG_H__
#define __RF24_CONFIG_H__

#include <stddef.h>
#include <QTypeInfo>

// Stuff that is normally provided by Arduino
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define _BV(x) (1<<(x))

typedef char const prog_char;
typedef quint16 prog_quint16;
#define PSTR(x) (x)
#define strlen_P strlen
#define PROGMEM
#define pgm_read_word(p) (*(p)) 

#endif // __RF24_CONFIG_H__
// vim:ai:cin:sts=2 sw=2 ft=cpp
