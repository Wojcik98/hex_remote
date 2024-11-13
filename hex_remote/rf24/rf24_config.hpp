
/*
 Copyright (C)
    2011            J. Coliz <maniacbug@ymail.com>
    2015-2019       TMRh20
    2015            spaniakos <spaniakos@gmail.com>
    2015            nerdralph
    2015            zador-blood-stained
    2016            akatran
    2017-2019       Avamander <avamander@gmail.com>
    2019            IkpeohaGodson
    2021            2bndy5
    2024            Wojcik98

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
*/

#ifndef RF24_CONFIG_H_
#define RF24_CONFIG_H_

/*** USER DEFINES:    ***/
#define FAILURE_HANDLING

/**
 * User access to internally used delay time (in microseconds) during
 * RF24::powerUp()
 * @warning This default value compensates for all supported hardware. Only
 * adjust this if you know your radio's hardware is, in fact, genuine and
 * reliable.
 */
#if !defined(RF24_POWERUP_DELAY)
// #define RF24_POWERUP_DELAY 5
#endif

/// A sentinel used to control fetching the pipe info in `RF24::available()`.
#define RF24_NO_FETCH_PIPE 0XFF

#endif // RF24_CONFIG_H_
