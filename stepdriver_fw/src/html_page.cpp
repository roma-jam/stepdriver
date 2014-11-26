/*
 * html_page.cpp
 *
 *  Created on: 25 но€б. 2014 г.
 *      Author: r.leonov
 */

#include "html_page.h"

const char *glidertrack_html[] = { "<html>                                                     \
        <head><title>Handycam Glidertrack</title></head>                                        \
        <body bgcolor=\"gray\" text=\"white\" style=\"text-shadow: 0px 6px 6px rgba(0,0,0,.5);\"> \
        <style>                                                                                 \
        .big_text {                                                                             \
            font-size: 85px;                                                                    \
        }                                                                                       \
        .button_size {                                                                          \
            width: 150px;                                                                       \
            height: 120px;                                                                  \
            color: grey;                                                                    \
            font-size: 28px;                                                                \
            text-shadow: 0px 2px 2px rgba(0,0,0,.5);                                        \
        }                                                                                   \
        .input {                                                                            \
            width: 240px;                                                                   \
            height: 120px;                                                                  \
            line-height: 40px;                                                              \
            text-align: center;                                                             \
            font-size: 28px;                                                                \
        }                                                                                   \
                                                                                            \
        .bord {                                                                             \
            border-color: white;                                                            \
            border-radius: 10px;                                                            \
        }                                                                                   \
        </style>                                                                            \
        <fieldset class=bord>                                                               \
            <legend align=\"center\">                                                         \
                <h4 class=big_text>Manual</h4>                                              \
            </legend>                                                                       \
                <form align = \"center\">                                                     \
                    <input class=input type=\"text\" name=\"speed\" size=\"10\" maxlength=\"10\" value=\"\" placeholder=\"Input slide speed\">&nbsp; \
                    <input class=button_size type=\"submit\" value=\"SLIDE\">   \
                </form> \
                <form align = \"center\"> \
                    <button class=button_size name=\"button\" type=\"text\" value=\"SetHome\">Set Home</button>&nbsp \
                    <button class=button_size name=\"button\" type=\"text\" value=\"Calibrate\">Calibrate</button>&nbsp \
                </form> \
                <form align = \"center\"> \
                    <button class=button_size name=\"button\" type=\"text\" value=\"Left\">Left</button>&nbsp \
                    <button class=button_size name=\"button\" type=\"text\" value=\"Rigth\">Right</button>&nbsp \
                    <button class=button_size name=\"button\" type=\"text\" value=\"Stop\">Stop</button>&nbsp \
                </form> \
        </fieldset> \
        <fieldset class=bord> \
            <legend align=\"center\"> \
                <h4 class=big_text>Timelapse</h4> \
            </legend> \
                <form align = \"center\"> \
                    <input class=input type=\"text\" name=\"time\" size=\"10\" maxlength=\"10\" value=\"\" placeholder=\"Input time\">&nbsp; \
                    <input class=button_size type=\"submit\" value=\"Minutes\"> \
                </form> \
                <form align=\"center\"> \
                    <button class=button_size name=\"button\" type=\"text\" value=\"StartTL\">START</button>&nbsp \
                </form> \
        </fieldset> \
        <fieldset class=bord> \
            <legend align=\"center\"> \
                <h4 class=big_text>Response</h4> \
            </legend> \
            <p align = \"center\"> \
                Bla Bla Bla \
            </p>        \
        </fieldset>         \
        <p style = \"position: fixed; bottom: 0; font-size: 35px;\">More information: \
            <a style = \"text-decoration: underline; color: #fff;\" href=\"http://www.Handycam.pro/\" >http://www.handycam.pro/</a>. \
        </p> \
        </body> \
        </html>"
};
