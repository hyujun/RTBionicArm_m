/*
 * devMouseObject.h
 *
 *  Created on: 2020. 3. 16.
 *      Author: parkjunho
 */

#ifndef DEVMOUSEOBJECT_H_
#define DEVMOUSEOBJECT_H_

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <linux/input.h>
#include <fcntl.h>

#include <stdarg.h>
#include <stdint.h>
#include <string.h>

#include <linux/fb.h>
#include <sys/mman.h>

#include <iostream>
#include <map>
#include <string>

using namespace std;

enum {
	VERBOSE    = true,
	MAX_WIDTH  = 1280,
	MAX_HEIGHT = 1024,
	VALUE_PRESSED  = 1,
	VALUE_RELEASED = 0,
	BITS_PER_BYTE  = 8,
	CURSOR_COLOR   = 0x00FF00,
};

struct cursor_t {
	int x, y;
};

struct mouse_t {
	struct cursor_t current;
	struct cursor_t pressed, released;
	//bool button_pressed;
	//bool button_released;
};

struct fb_t {
	int fd;
	unsigned char *fp;
	int width, height;   /* display resolution */
	long screen_size;    /* screen data size (byte) */
	int line_length;     /* line length (byte) */
	int bytes_per_pixel;
	int bits_per_pixel;
};



/* logging functions */
enum loglevel_t {
	DEBUG = 0,
	WARN,
	ERROR,
	FATAL,
};

class devMouseObject {

public:
	devMouseObject();
	virtual ~devMouseObject();

	void loadMouseEvents(void);

	int Activate();
	void Deactivate();
	void logging(enum loglevel_t loglevel, const char *format, ...);
	int my_ceil(int val, int div);
	bool fb_init(struct fb_t *fb);
	void fb_die(struct fb_t *fb);
	void fb_draw(struct fb_t *fb, struct cursor_t *cursor, uint32_t color);
	void init_mouse(struct mouse_t *mouse);

	void print_event(void);
	void print_event(struct input_event *ie);

	void print_mouse_state(void);
	void print_mouse_state(struct mouse_t *mouse);

	void cursor(struct input_event *ie, struct mouse_t *mouse);
	void button(struct input_event *ie, struct mouse_t *mouse);

	int getMousefd(void)
	{
		return mouse_fd;
	}

	struct input_event getInputEvent(void)
	{
		return m_ie;
	}

	struct mouse_t getMouse(void)
	{
		return m_mouse;
	}

	struct fb_t getMousefb(void)
	{
		return m_fb;
	}

private:
	map<int, string> m_ev_type;
	map<int, string> m_ev_rel;
	map<int, string> m_ev_key;
	map<int, string> m_ev_syn;

	int mouse_fd=-1;
	const char *dev;
	struct input_event m_ie;
	struct mouse_t m_mouse;
	struct fb_t m_fb;

	//const char *mouse_dev = "/dev/input/event8";
	//const char *mouse_dev = "/dev/input/event9";
	const char *mouse_dev = "/dev/input/mice";
	const char *fb_dev    = "/dev/fb0";

};

#endif /* DEVMOUSEOBJECT_H_ */
