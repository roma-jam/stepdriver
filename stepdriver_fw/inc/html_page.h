/*
 * html_page.h
 *
 *  Created on: 25 но€б. 2014 г.
 *      Author: r.leonov
 */

#ifndef HTML_PAGE_H_
#define HTML_PAGE_H_

const char get_response_header[] = "\
HTTP/1.1 200 OK\r\n\
Content-type: text/html\r\n\
Content-length: ";

const char get_response_ok[] = "\
HTTP/1.1 204 OK\r\n\
";
extern char index_html[];

#endif /* HTML_PAGE_H_ */
