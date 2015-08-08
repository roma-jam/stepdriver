/*
 * html_page.cpp
 *
 *  Created on: 25 но€б. 2014 г.
 *      Author: r.leonov
 */

#include "html_page.h"

const char get_response_header[] = "\
HTTP/1.1 200 OK\r\n\
Content-type: text/html\r\n\
Content-length: ";

const char get_response_ok[] = "\
HTTP/1.1 204 OK\r\n\
";

const char glidertrack[] = "\
<html>\r\n\
<body bgcolor=\"#333\" text=\"white\" style=\"text-shadow: 0px 6px 6px rgba(0,0,0,.5);\">\r\n\
<style>\r\n\
.big_text {\r\n\
	font-size: 450%;\r\n\
}\r\n\
.button_size {\r\n\
	width: 23%;\r\n\
	height: 9%;\r\n\
	color: grey;\r\n\
	font-size: 230%;\r\n\
	text-shadow: 1px 1px 4px rgba(0,0,0,.5);\r\n\
}\r\n\
.input {\r\n\
	width: 47%;\r\n\
	height: 9%;\r\n\
	text-align: center;\r\n\
	font-size: 330%;\r\n\
}\r\n\
.bord {\r\n\
	border-color: white;\r\n\
	border-radius: 20px;\r\n\
}\r\n\
</style>\r\n\
<fieldset class=bord>\r\n\
	<legend align=\"center\">\r\n\
		<h4 class=big_text>Manual</h4>\r\n\
	</legend>\r\n\
		<form align =\"center\">\r\n\
			<input class=input type=\"text\" name=\"speed\" size=\"10\" maxlength=\"10\" value=\"\" placeholder=\"Input speed...\">&nbsp;\r\n\
			<input class=button_size type=\"submit\" value=\"Slide\">\r\n\
		</form>\r\n\
		<form align =\"center\">\r\n\
			<button class=button_size name=\"button\" type=\"text\" value=\"SetHome\">Set Home</button>&nbsp\r\n\
			<button class=button_size name=\"button\" type=\"text\" value=\"Calibrate\">Calibrate</button>&nbsp\r\n\
			<button class=button_size name=\"button\" type=\"text\" value=\"GoHome\">Go Home</button>&nbsp\r\n\
		</form>\r\n\
		<form align =\"center\">\r\n\
			<button class=button_size name=\"button\" type=\"text\" value=\"Left\">Left</button>&nbsp\r\n\
			<button class=button_size name=\"button\" type=\"text\" value=\"Stop\">Stop</button>&nbsp\r\n\
			<button class=button_size name=\"button\" type=\"text\" value=\"Rigth\">Right</button>&nbsp\r\n\
		</form>\r\n\
</fieldset>\r\n\
<fieldset class=bord>\r\n\
	<legend align=\"center\">\r\n\
		<h4 class=big_text>Timelapse</h4>\r\n\
	</legend>\r\n\
		<form align =\"center\">\r\n\
			<input class=input type=\"text\" name=\"time\" size=\"10\" maxlength=\"10\" value=\"\" placeholder=\"Input time...\">&nbsp;\r\n\
			<input class=button_size type=\"submit\" value=\"Minutes\">\r\n\
		</form>\r\n\
		<form align=\"center\">\r\n\
			<button class=button_size name=\"button\" type=\"text\" value=\"StartTL\">Start</button>&nbsp\r\n\
		</form>\r\n\
</fieldset>\r\n\
<fieldset class=bord>\r\n\
	<legend align=\"center\">\r\n\
		<h4 class=big_text>Response</h4>\r\n\
	</legend>\r\n\
	<p align =\"center\" style=\"font-size: 300%\">\r\n\
		OK, FAILURE, SETUP\r\n\
	</p></fieldset>\r\n\
<p style =\"position: fixed; bottom: 0; font-size: 35px;\">More information:\r\n\
	<a style =\"text-decoration: underline; color: #fff;\" href=\"www.Handycam.pro/\" >http://www.handycam.pro/</a>.\r\n\
</p>\r\n\
</body>\r\n\
</html>\r\n";
