/*
 * devMouseObject.cpp
 *
 *  Created on: 2020. 3. 16.
 *      Author: parkjunho
 */

#include "devMouseObject.h"

devMouseObject::devMouseObject() {
	this->loadMouseEvents();

}

devMouseObject::~devMouseObject() {

}

int devMouseObject::Activate()
{
	mouse_fd = open(mouse_dev, O_RDONLY);
	if(mouse_fd == -1)
	{
		perror("opening device(mouse)");
		return -1;
	}

	init_mouse(&m_mouse);
	fb_init(&m_fb);
	return 1;
}

void devMouseObject::Deactivate()
{
	fb_die(&m_fb);
	close(mouse_fd);
}

void devMouseObject::loadMouseEvents(void)
{
	m_ev_type.clear();
	m_ev_type.insert(make_pair(EV_SYN, 		"EV_SYN"));
	m_ev_type.insert(make_pair(EV_KEY, 		"EV_KEY"));
	m_ev_type.insert(make_pair(EV_REL, 		"EV_REL"));
	m_ev_type.insert(make_pair(EV_ABS, 		"EV_ABS"));
	m_ev_type.insert(make_pair(EV_MSC, 		"EV_MSC"));
	m_ev_type.insert(make_pair(EV_SW, 		"EV_SW"));
	m_ev_type.insert(make_pair(EV_LED,		"EV_LED"));
	m_ev_type.insert(make_pair(EV_SND, 		"EV_SND"));
	m_ev_type.insert(make_pair(EV_REP, 		"EV_REP"));
	m_ev_type.insert(make_pair(EV_FF, 		"EV_FF"));
	m_ev_type.insert(make_pair(EV_PWR, 		"EV_PWR"));
	m_ev_type.insert(make_pair(EV_FF_STATUS, "EV_FF_STATUS"));
	m_ev_type.insert(make_pair(EV_MAX, 		"EV_MAX"));

	m_ev_syn.clear();
	m_ev_syn.insert(make_pair(SYN_REPORT, 		"SYN_REPORT"));
	m_ev_syn.insert(make_pair(SYN_CONFIG, 		"SYN_CONFIG"));
	m_ev_syn.insert(make_pair(SYN_MT_REPORT, 	"SYN_MT_REPORT"));
	m_ev_syn.insert(make_pair(SYN_DROPPED, 		"SYN_DROPPED"));

	m_ev_rel.clear();
	m_ev_rel.insert(make_pair(REL_X, 		"REL_X"));
	m_ev_rel.insert(make_pair(REL_Y, 		"REL_Y"));
	m_ev_rel.insert(make_pair(REL_Z, 		"REL_Z"));
	m_ev_rel.insert(make_pair(REL_RX, 		"REL_RX"));
	m_ev_rel.insert(make_pair(REL_RY, 		"REL_RY"));
	m_ev_rel.insert(make_pair(REL_RZ, 		"REL_RZ"));
	m_ev_rel.insert(make_pair(REL_HWHEEL, 	"REL_HWHEEL"));
	m_ev_rel.insert(make_pair(REL_DIAL, 	"REL_DIAL"));
	m_ev_rel.insert(make_pair(REL_WHEEL, 	"REL_WHEEL"));
	m_ev_rel.insert(make_pair(REL_MISC, 	"REL_MISC"));
	m_ev_rel.insert(make_pair(REL_MAX, 		"REL_MAX"));

	m_ev_key.clear();
	m_ev_key.insert(make_pair(BTN_LEFT, 	"BTN_LEFT"));
	m_ev_key.insert(make_pair(BTN_RIGHT, 	"BTN_RIGHT"));
	m_ev_key.insert(make_pair(BTN_MIDDLE, 	"BTN_MIDDLE"));
	m_ev_key.insert(make_pair(BTN_SIDE, 	"BTN_SIDE"));
	m_ev_key.insert(make_pair(BTN_EXTRA, 	"BTN_EXTRA"));
	m_ev_key.insert(make_pair(BTN_FORWARD, 	"BTN_FORWARD"));
	m_ev_key.insert(make_pair(BTN_BACK, 	"BTN_BACK"));
	m_ev_key.insert(make_pair(BTN_TASK, 	"BTN_TASK"));
	m_ev_key.insert(make_pair(KEY_MAX, 		"KEY_MAX"));

}

void devMouseObject::logging(enum loglevel_t loglevel, const char *format, ...)
{
    va_list arg;
    static const char *loglevel2str[] = {
        [DEBUG] = "DEBUG",
        [WARN]  = "WARN",
        [ERROR] = "ERROR",
        [FATAL] = "FATAL",
    };

    /* debug message is available on verbose mode */
    if ((loglevel == DEBUG) && (VERBOSE == false))
        return;

    fprintf(stderr, ">>%s<<\t", loglevel2str[loglevel]);

    va_start(arg, format);
    vfprintf(stderr, format, arg);
    va_end(arg);
}

int devMouseObject::my_ceil(int val, int div)
{
	if(div == 0)
	{
		return 0;
	}
	else
	{
		return (val + div - 1)/div;
	}
}

bool devMouseObject::fb_init(fb_t *fb)
{
	struct fb_fix_screeninfo finfo;
	struct fb_var_screeninfo vinfo;

	if ((fb->fd = open(fb_dev, O_RDWR)) < 0) {
		perror("open");
		return false;
	}

	if (ioctl(fb->fd, FBIOGET_FSCREENINFO, &finfo)) {
		logging(ERROR, "ioctl: FBIOGET_FSCREENINFO failed\n");
		return false;
	}

	if (ioctl(fb->fd, FBIOGET_VSCREENINFO, &vinfo)) {
		logging(ERROR, "ioctl: FBIOGET_VSCREENINFO failed\n");
		return false;
	}

	fb->width  = vinfo.xres;
	fb->height = vinfo.yres;
	fb->screen_size = finfo.smem_len;
	fb->line_length = finfo.line_length;
	fb->bits_per_pixel  = vinfo.bits_per_pixel;
	fb->bytes_per_pixel = my_ceil(fb->bits_per_pixel, BITS_PER_BYTE);

	if ((fb->fp = (unsigned char *) mmap(0, fb->screen_size,
		PROT_WRITE | PROT_READ, MAP_SHARED, fb->fd, 0)) == MAP_FAILED)
		return false;

	return true;
}

void devMouseObject::fb_die(fb_t *fb)
{
	munmap(fb->fp, fb->screen_size);
	close(fb->fd);
}

void devMouseObject::fb_draw(fb_t*fb, cursor_t *cursor, uint32_t color)
{
	memcpy(fb->fp + cursor->y * fb->line_length + cursor->x * fb->bytes_per_pixel, &color, fb->bytes_per_pixel);
}

void devMouseObject::init_mouse(mouse_t *mouse)
{
	mouse->current.x  = mouse->current.y  = 0;
	mouse->pressed.x  = mouse->pressed.y  = 0;
	mouse->released.x = mouse->released.y = 0;
	//mouse->button_pressed  = false;
	//mouse->button_released = false;
}

void devMouseObject::print_event(void)
{
	switch (m_ie.type)
	{
	case EV_SYN:
		fprintf(stderr, "\ntime:%ld.%06ld\ttype:%s\tcode:%s\tvalue:%d",
			m_ie.time.tv_sec, m_ie.time.tv_usec, m_ev_type.find(m_ie.type)->second.c_str(),
			m_ev_syn.find(m_ie.code)->second.c_str(), m_ie.value);
		break;
	case EV_REL:
		fprintf(stderr, "\ntime:%ld.%06ld\ttype:%s\tcode:%s\tvalue:%d",
			m_ie.time.tv_sec, m_ie.time.tv_usec, m_ev_type.find(m_ie.type)->second.c_str(),
			m_ev_rel.find(m_ie.code)->second.c_str(), m_ie.value);
		break;
	case EV_KEY:
		fprintf(stderr, "\ntime:%ld.%06ld\ttype:%s\tcode:%s\tvalue:%d",
			m_ie.time.tv_sec, m_ie.time.tv_usec, m_ev_type.find(m_ie.type)->second.c_str(),
			m_ev_key.find(m_ie.code)->second.c_str(), m_ie.value);
		break;
	default:
		break;
	}
}

void devMouseObject::print_event(struct input_event *ie)
{
	switch (ie->type)
	{
	case EV_SYN:
		fprintf(stderr, "\ntime:%ld.%06ld\ttype:%s\tcode:%s\tvalue:%d",
			ie->time.tv_sec, ie->time.tv_usec, m_ev_type.find(ie->type)->second.c_str(),
			m_ev_syn.find(ie->code)->second.c_str(), ie->value);
		break;
	case EV_REL:
		fprintf(stderr, "\ntime:%ld.%06ld\ttype:%s\tcode:%s\tvalue:%d",
			ie->time.tv_sec, ie->time.tv_usec, m_ev_type.find(ie->type)->second.c_str(),
			m_ev_rel.find(ie->code)->second.c_str(), ie->value);
		break;
	case EV_KEY:
		fprintf(stderr, "\ntime:%ld.%06ld\ttype:%s\tcode:%s\tvalue:%d",
			ie->time.tv_sec, ie->time.tv_usec, m_ev_type.find(ie->type)->second.c_str(),
			m_ev_key.find(ie->code)->second.c_str(), ie->value);
		break;
	default:
		break;
	}
}

void devMouseObject::print_mouse_state(void)
{
	fprintf(stderr, "\n\033[1;1H\033[2Kcurrent(%d, %d) pressed(%d, %d) released(%d, %d)",
			m_mouse.current.x,m_mouse.current.y,
			m_mouse.pressed.x, m_mouse.pressed.y,
			m_mouse.released.x, m_mouse.released.y);
}

void devMouseObject::print_mouse_state(struct mouse_t *mouse)
{
	fprintf(stderr, "\n\033[1;1H\033[2Kcurrent(%d, %d) pressed(%d, %d) released(%d, %d)",
		mouse->current.x, mouse->current.y,
		mouse->pressed.x, mouse->pressed.y,
		mouse->released.x, mouse->released.y);
}

void devMouseObject::cursor(struct input_event *ie, struct mouse_t *mouse)
{
	if (ie->code == REL_X)
		mouse->current.x += ie->value;

	if (mouse->current.x < 0)
		mouse->current.x = 0;
	else if (mouse->current.x >= MAX_WIDTH)
		mouse->current.x = MAX_WIDTH - 1;

	if (ie->code == REL_Y)
		mouse->current.y += ie->value;

	if (mouse->current.y < 0)
		mouse->current.y = 0;
	else if (mouse->current.y >= MAX_HEIGHT)
		mouse->current.y = MAX_HEIGHT - 1;
}

void devMouseObject::button(struct input_event *ie, struct mouse_t *mouse)
{
	if (ie->code != BTN_LEFT)
		return;

	if (ie->value == VALUE_PRESSED)
		mouse->pressed = mouse->current;

	if (ie->value == VALUE_RELEASED)
		mouse->released = mouse->current;
}


/*
 * example
 *
int main(int argc, char *argv[])
{
	int fd;
	struct input_event ie;
	struct mouse_t mouse;
	struct fb_t fb;
	const char *dev;

	dev = (argc > 1) ? argv[1]: mouse_dev;

	if ((fd = open(dev, O_RDONLY)) == -1) {
		perror("opening device");
		exit(EXIT_FAILURE);
	}

	init_mouse(&mouse);
	fb_init(&fb);

	while (read(fd, &ie, sizeof(struct input_event))) {
		//print_event(&ie);
		print_mouse_state(&mouse);
		fb_draw(&fb, &mouse.current, CURSOR_COLOR);

		if (event_handler[ie.type] == EV_REL)
			cursor(&ie, &mouse);
		elseif(event_handler[ie.type] == EV_KEY)
			button(&ie, &mouse);
	}

	fb_die(&fb);

	close(fd);

	return EXIT_SUCCESS;
}
 *
 *
 */
